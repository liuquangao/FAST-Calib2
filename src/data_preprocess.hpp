/*
Developer: Chunran Zheng <zhengcr@connect.hku.hk>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef DATA_PREPROCESS_HPP
#define DATA_PREPROCESS_HPP

#include <Eigen/Core>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "bag_reader.hpp"
#include "common_lib.h"

using namespace std;

class DataPreprocess
{
public:
    // 改成带线号的点云
    pcl::PointCloud<Common::Point>::Ptr cloud_input_;
    cv::Mat img_input_;
    LiDARType lidar_type_{LiDARType::Unknown};
    LiDARType lidarType() const { return lidar_type_; }

    DataPreprocess(Params &params)
        : cloud_input_(new pcl::PointCloud<Common::Point>)
    {
        // 读图像
        img_input_ = cv::imread(params.image_path, cv::IMREAD_UNCHANGED);
        if (img_input_.empty())
        {
            ROS_ERROR_STREAM("Loading the image " << params.image_path << " failed");
            return;
        }

        // 累计读取 rosbag2 中的全部点云（失败时点云为空、类型为 Unknown）
        loadCloudFromBag(params.bag_path, params.lidar_topic, cloud_input_, lidar_type_);
    }
};

typedef std::shared_ptr<DataPreprocess> DataPreprocessPtr;

#endif // DATA_PREPROCESS_HPP
