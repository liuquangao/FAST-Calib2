/*
ROS 2 bag loading for FAST-Calib2.

Reads every message of one LiDAR topic from a rosbag2 (directory or .db3 file)
and accumulates it into a Common::Point cloud. Supports sensor_msgs/PointCloud2
and, when built against livox_ros_driver2, livox_ros_driver2/CustomMsg.
*/

#ifndef BAG_READER_HPP
#define BAG_READER_HPP

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#ifdef FAST_CALIB_HAVE_LIVOX
#include <livox_ros_driver2/msg/custom_msg.hpp>
#endif
#include <cstring>
#include <functional>
#include <limits>
#include <string>
#include "common_lib.h"

enum class LiDARType : int {
    Unknown = 0,
    Solid   = 1,   // 固态（如 Livox）
    Mech    = 2    // 机械式多线
};

namespace bag_reader_detail
{
inline const sensor_msgs::msg::PointField* findField(const sensor_msgs::msg::PointCloud2& msg,
                                                     const std::string& name)
{
    for (const auto& f : msg.fields)
    {
        if (f.name == name) return &f;
    }
    return nullptr;
}

// 按字段实际数据类型读取，兼容 float32 / uint8 / uint16 等不同驱动的 intensity、ring 定义
inline double readField(const std::uint8_t* point, const sensor_msgs::msg::PointField& f)
{
    const std::uint8_t* p = point + f.offset;
    using PF = sensor_msgs::msg::PointField;
    switch (f.datatype)
    {
        case PF::INT8:    { std::int8_t v;   std::memcpy(&v, p, sizeof(v)); return v; }
        case PF::UINT8:   { std::uint8_t v;  std::memcpy(&v, p, sizeof(v)); return v; }
        case PF::INT16:   { std::int16_t v;  std::memcpy(&v, p, sizeof(v)); return v; }
        case PF::UINT16:  { std::uint16_t v; std::memcpy(&v, p, sizeof(v)); return v; }
        case PF::INT32:   { std::int32_t v;  std::memcpy(&v, p, sizeof(v)); return v; }
        case PF::UINT32:  { std::uint32_t v; std::memcpy(&v, p, sizeof(v)); return v; }
        case PF::FLOAT32: { float v;         std::memcpy(&v, p, sizeof(v)); return v; }
        case PF::FLOAT64: { double v;        std::memcpy(&v, p, sizeof(v)); return v; }
        default: return std::numeric_limits<double>::quiet_NaN();
    }
}

inline bool appendPointCloud2(const sensor_msgs::msg::PointCloud2& msg, std::uint32_t scan_id,
                              pcl::PointCloud<Common::Point>& cloud, LiDARType& type)
{
    const auto* fx = findField(msg, "x");
    const auto* fy = findField(msg, "y");
    const auto* fz = findField(msg, "z");
    if (!fx || !fy || !fz)
    {
        ROS_ERROR("PointCloud2 message has no x/y/z fields.");
        return false;
    }
    const auto* fring = findField(msg, "ring");
    const auto* fint = findField(msg, "intensity");
    if (!fint) fint = findField(msg, "reflectivity");

    if (type == LiDARType::Unknown)
    {
        type = fring ? LiDARType::Mech : LiDARType::Solid;
    }

    for (std::uint32_t row = 0; row < msg.height; ++row)
    {
        for (std::uint32_t col = 0; col < msg.width; ++col)
        {
            const std::uint8_t* pt = msg.data.data() + row * msg.row_step + col * msg.point_step;
            Common::Point p;
            p.x = static_cast<float>(readField(pt, *fx));
            p.y = static_cast<float>(readField(pt, *fy));
            p.z = static_cast<float>(readField(pt, *fz));
            p.intensity = fint ? static_cast<float>(readField(pt, *fint)) : 0.0f;
            p.ring = fring ? static_cast<std::uint16_t>(readField(pt, *fring)) : 0xFFFF;
            p.scan_id = scan_id;
            cloud.push_back(p);
        }
    }
    return true;
}

#ifdef FAST_CALIB_HAVE_LIVOX
inline void appendLivox(const livox_ros_driver2::msg::CustomMsg& msg, std::uint32_t scan_id,
                        pcl::PointCloud<Common::Point>& cloud)
{
    for (const auto& lp : msg.points)
    {
        Common::Point p;
        p.x = lp.x;
        p.y = lp.y;
        p.z = lp.z;
        p.intensity = static_cast<float>(lp.reflectivity);
        p.ring = static_cast<std::uint16_t>(lp.line);
        p.scan_id = scan_id;
        cloud.push_back(p);
    }
}
#endif

template <typename MsgT>
inline MsgT deserialize(const rosbag2_storage::SerializedBagMessage& bag_msg)
{
    static rclcpp::Serialization<MsgT> serializer;
    rclcpp::SerializedMessage serialized(*bag_msg.serialized_data);
    MsgT msg;
    serializer.deserialize_message(&serialized, &msg);
    return msg;
}
}  // namespace bag_reader_detail

// 从 rosbag2 中读取指定 LiDAR topic，并统一转换为 Common::Point 点云。
// 失败时 cloud 为空、detected_type 为 Unknown。
inline bool loadCloudFromBag(const std::string& bag_path,
                             const std::string& lidar_topic,
                             pcl::PointCloud<Common::Point>::Ptr cloud,
                             LiDARType& detected_type)
{
    using namespace bag_reader_detail;
    cloud->clear();
    detected_type = LiDARType::Unknown;

    rosbag2_cpp::Reader reader;
    try
    {
        reader.open(bag_path);
    }
    catch (const std::exception& e)
    {
        ROS_ERROR_STREAM("Failed to open rosbag2 " << bag_path << ": " << e.what());
        return false;
    }

    std::string topic_type;
    std::string available;
    for (const auto& meta : reader.get_all_topics_and_types())
    {
        available += " " + meta.name + " (" + meta.type + ")";
        if (meta.name == lidar_topic) topic_type = meta.type;
    }
    if (topic_type.empty())
    {
        ROS_ERROR_STREAM("Topic " << lidar_topic << " not found in " << bag_path
                         << ". Available:" << available);
        return false;
    }

    // Pick the decoder once; every message on the topic has the same type.
    std::function<bool(const rosbag2_storage::SerializedBagMessage&, std::uint32_t)> append;
    if (topic_type == "sensor_msgs/msg/PointCloud2")
    {
        append = [&](const rosbag2_storage::SerializedBagMessage& m, std::uint32_t id) {
            return appendPointCloud2(deserialize<sensor_msgs::msg::PointCloud2>(m), id, *cloud,
                                     detected_type);
        };
    }
#ifdef FAST_CALIB_HAVE_LIVOX
    else if (topic_type == "livox_ros_driver2/msg/CustomMsg")
    {
        detected_type = LiDARType::Solid;
        append = [&](const rosbag2_storage::SerializedBagMessage& m, std::uint32_t id) {
            appendLivox(deserialize<livox_ros_driver2::msg::CustomMsg>(m), id, *cloud);
            return true;
        };
    }
#endif
    else
    {
        ROS_ERROR_STREAM("Unsupported LiDAR message type " << topic_type << " on " << lidar_topic);
        return false;
    }

    rosbag2_storage::StorageFilter filter;
    filter.topics.push_back(lidar_topic);
    reader.set_filter(filter);

    std::uint32_t scan_id = 0;
    while (reader.has_next())
    {
        if (!append(*reader.read_next(), scan_id++))
        {
            cloud->clear();
            detected_type = LiDARType::Unknown;
            return false;
        }
    }

    ROS_INFO("Loaded %u messages, %zu points from %s", scan_id, cloud->size(), bag_path.c_str());
    if (cloud->empty()) detected_type = LiDARType::Unknown;
    return !cloud->empty();
}

#endif  // BAG_READER_HPP
