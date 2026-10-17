/* 
Developer: Chunran Zheng <zhengcr@connect.hku.hk>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "qr_detect.hpp"
#include "lidar_detect.hpp"
#include "data_preprocess.hpp"

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr nh = fast_calib_compat::makeNode("fast_calib");

    // 读取参数
    Params params = loadParameters(nh);
    std::string mounting_error;
    if (!validateLidarMountAxes(params.lidar_forward_axis, params.lidar_up_axis,
                                mounting_error))
    {
      ROS_ERROR_STREAM("[Main] Invalid LiDAR mounting-axis configuration: "
                       << mounting_error);
      return 1;
    }

    std::string output_error;
    if (!ensureDirectoryTree(params.output_path, output_error))
    {
      ROS_ERROR_STREAM("[Main] Invalid output directory: " << output_error);
      return 1;
    }

    // Load the inputs before constructing detectors so camera calibration can
    // be checked against the actual image resolution.
    DataPreprocessPtr dataPreprocessPtr;
    dataPreprocessPtr.reset(new DataPreprocess(params));

    // 读取图像和点云
    cv::Mat img_input = dataPreprocessPtr->img_input_;
    pcl::PointCloud<Common::Point>::Ptr cloud_input = dataPreprocessPtr->cloud_input_;
    if (img_input.empty())
    {
      ROS_ERROR_STREAM("[Main] Image is empty. Check image_path: " << params.image_path);
      return 1;
    }
    if (!cloud_input || cloud_input->empty())
    {
      ROS_ERROR_STREAM("[Main] Point cloud is empty. Check bag_path and lidar_topic: "
                       << params.bag_path << ", " << params.lidar_topic);
      return 1;
    }
    if (dataPreprocessPtr->lidar_type_ == LiDARType::Unknown)
    {
      ROS_ERROR_STREAM("[Main] Unknown LiDAR message type. Check lidar_topic: "
                       << params.lidar_topic);
      return 1;
    }

    std::string camera_error;
    if (!validateCameraCalibrationForImage(params, img_input.cols, img_input.rows,
                                           camera_error))
    {
      ROS_ERROR_STREAM("[Main] Invalid camera calibration: " << camera_error);
      return 1;
    }

    // 初始化 QR 检测和 LiDAR 检测
    QRDetectPtr qrDetectPtr;
    qrDetectPtr.reset(new QRDetect(nh, params));

    LidarDetectPtr lidarDetectPtr;
    lidarDetectPtr.reset(new LidarDetect(nh, params));
    
    // 检测 QR 码
    PointCloud<PointXYZ>::Ptr qr_center_cloud(new PointCloud<PointXYZ>);
    qr_center_cloud->reserve(4);
    qrDetectPtr->detect_qr(img_input, qr_center_cloud);
    if (qr_center_cloud->size() != TARGET_NUM_CIRCLES)
    {
      ROS_ERROR_STREAM("[Main] Expected " << TARGET_NUM_CIRCLES
                       << " camera target centers, got " << qr_center_cloud->size() << ".");
      return 1;
    }

    // 检测 LiDAR 数据
    PointCloud<PointXYZ>::Ptr lidar_center_cloud(new PointCloud<PointXYZ>);
    lidar_center_cloud->reserve(4);
    
    switch (dataPreprocessPtr->lidar_type_)
    {
        case LiDARType::Solid:
            lidarDetectPtr->detect_solid_lidar(cloud_input, lidar_center_cloud);
            break;

        case LiDARType::Mech:
            lidarDetectPtr->detect_mech_lidar(cloud_input, lidar_center_cloud);
            break;

        default:
            std::cerr << BOLDYELLOW 
                    << "[Main] Unknown LiDAR type." 
                    << RESET << std::endl;
            break;
    }
    if (lidar_center_cloud->size() != TARGET_NUM_CIRCLES)
    {
      ROS_ERROR_STREAM("[Main] Expected " << TARGET_NUM_CIRCLES
                       << " LiDAR target centers, got " << lidar_center_cloud->size() << ".");
      return 1;
    }

    // 对 QR 和 LiDAR 检测到的圆心进行排序
    PointCloud<PointXYZ>::Ptr qr_centers(new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr lidar_centers(new PointCloud<PointXYZ>);
    if (!sortPatternCenters(qr_center_cloud, qr_centers, "camera") ||
        !sortPatternCenters(lidar_center_cloud, lidar_centers, "lidar",
                            params.lidar_forward_axis, params.lidar_up_axis))
    {
      ROS_ERROR("[Main] Failed to sort target centers. Check the LiDAR mounting-axis configuration.");
      return 1;
    }

    validateTargetGeometry(qr_centers, params.delta_width_circles, params.delta_height_circles, "QR");
    validateTargetGeometry(lidar_centers, params.delta_width_circles, params.delta_height_circles, "LiDAR");

    // 保存中间结果：排序后的 LiDAR 圆心和 QR 圆心
    saveTargetHoleCenters(lidar_centers, qr_centers, params);

    // 计算外参
    Eigen::Matrix4f transformation;
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> svd;
    svd.estimateRigidTransformation(*lidar_centers, *qr_centers, transformation);

    // 将 LiDAR 点云转换到 QR 码坐标系
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_lidar_centers(new pcl::PointCloud<pcl::PointXYZ>);
    aligned_lidar_centers->reserve(lidar_centers->size());
    alignPointCloud(lidar_centers, aligned_lidar_centers, transformation);
    
    double rmse = computeRMSE(qr_centers, aligned_lidar_centers);
    if (rmse > 0) 
    {
      std::cout << BOLDYELLOW << "[Result] RMSE: " << BOLDRED << std::fixed << std::setprecision(4)
      << rmse << " m" << RESET << std::endl;
    }

    std::cout << BOLDYELLOW << "[Result] Single-scene calibration: extrinsic parameters T_cam_lidar = " << RESET << std::endl;
    std::cout << BOLDCYAN << std::fixed << std::setprecision(6) << transformation << RESET << std::endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    projectPointCloudToImage(cloud_input, transformation, qrDetectPtr->cameraMatrix_, qrDetectPtr->distCoeffs_, img_input, colored_cloud);

    saveCalibrationResults(params, transformation, colored_cloud, qrDetectPtr->imageCopy_);

    auto colored_cloud_pub = nh->create_publisher<sensor_msgs::msg::PointCloud2>("colored_cloud", 1);
    auto aligned_lidar_centers_pub = nh->create_publisher<sensor_msgs::msg::PointCloud2>("aligned_lidar_centers", 1);

    // 主循环：每秒发布一次检测结果和中间点云，便于在 RViz 中查看
    std_msgs::msg::Header header;
    header.frame_id = "map";
    auto publish = [&header](const auto& pub, const auto& cloud)
    {
      sensor_msgs::msg::PointCloud2 msg;
      pcl::toROSMsg(*cloud, msg);
      msg.header = header;
      pub->publish(msg);
    };

    rclcpp::Rate rate(1);
    while (rclcpp::ok()) 
    {
      if (DEBUG) 
      {
        header.stamp = nh->now();
        publish(qrDetectPtr->qr_pub_, qr_centers);
        publish(lidarDetectPtr->center_pub_, lidar_centers);
        publish(lidarDetectPtr->filtered_pub_, lidarDetectPtr->getFilteredCloud());
        publish(lidarDetectPtr->plane_pub_, lidarDetectPtr->getPlaneCloud());
        publish(lidarDetectPtr->annulus_pub_, lidarDetectPtr->getAnnulusOriginalCloud());
        publish(lidarDetectPtr->boundary_pub_, lidarDetectPtr->getBoundaryOriginalCloud());
        publish(lidarDetectPtr->aligned_pub_, lidarDetectPtr->getAlignedCloud());
        publish(lidarDetectPtr->edge_pub_, lidarDetectPtr->getEdgeCloud());
        publish(lidarDetectPtr->center_z0_pub_, lidarDetectPtr->getCenterZ0Cloud());
        publish(aligned_lidar_centers_pub, aligned_lidar_centers);
        publish(colored_cloud_pub, colored_cloud);
      }
      rclcpp::spin_some(nh);
      rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
