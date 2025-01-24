/**
 * @file rosbag_listener.cpp
 * @author Elaina (1463967532@qq.com)
 * @brief 这是ros2 bag辅助节点,接受ros2 bag原始消息并打上当前时间戳后发布
 * @version 0.1
 * @date 2025-01-23
 *
 * @copyright Copyright (c) 2025
 *
 */
#include <fmt/core.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
class RosbagListener : public rclcpp::Node
{
public:
    RosbagListener(const rclcpp::NodeOptions &options) : Node("rosbag_listener_node", options)
    {
        this->declare_parameter("topic_names", std::vector<std::string>());
        this->declare_parameter("topic_types", std::vector<std::string>());
        this->declare_parameter("topic_suffix", "/bag");
        auto topic_names = this->get_parameter("topic_names").as_string_array();
        auto topic_types = this->get_parameter("topic_types").as_string_array();
        // auto topic_suffix = this->get_parameter("topic_suffix").as_string();
        fmt::print("rosbag_listener_node has started\n");
        // RCLCPP_ERROR(this->get_logger(), "topic_names and topic_types size not equal");
        if (topic_names.size() != topic_types.size())
        {
            RCLCPP_ERROR(this->get_logger(), "topic_names and topic_types size not equal");
            return;
        }
        for (size_t i = 0; i < topic_names.size(); i++)
        {
            _topic_name_type_map[topic_names[i]] = topic_types[i];
            fmt::print("topic_name: {}, topic_type: {}\n", topic_names[i], topic_types[i]);
        }
        for (auto &topic_name_type : _topic_name_type_map)
        {
            auto topic_name = topic_name_type.first;
            auto topic_type = topic_name_type.second;
            auto topic_suffix = this->get_parameter("topic_suffix").as_string();
            // 通过话题名称与类型动态创建发布者与订阅者
            // 发布者是ros2包原始话题名
            // 订阅者是ros2包原始话题名+后缀(ros2bag重命名的话题)
            _topic_publish_map[topic_name] = this->create_generic_publisher(topic_name, topic_type, 10);
            _topic_subscribe_map[topic_name + topic_suffix] =
                this->create_generic_subscription(topic_name + topic_suffix, topic_type, 10, [this, topic_name](std::shared_ptr<rclcpp::SerializedMessage> message)
                                                  { callback(message, topic_name); });
        }
    }
    ~RosbagListener()
    {
    }

private:
    /**
     * @brief 收到原始消息后的回调函数
     *
     * @param message
     * @param topic_name
     */
    void callback(std::shared_ptr<rclcpp::SerializedMessage> message, std::string topic_name)
    {
        auto now_time = this->now();
        if (_topic_name_type_map[topic_name] == "sensor_msgs/msg/PointCloud2")
        {
            // 反序列化
            auto pointcloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
            rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization;
            serialization.deserialize_message(message.get(), pointcloud_msg.get());
            pointcloud_msg->header.stamp = now_time;
            // 序列化
            rclcpp::SerializedMessage serialized_msg;
            serialization.serialize_message(pointcloud_msg.get(), &serialized_msg);
            _topic_publish_map[topic_name]->publish(serialized_msg);
        }
        else if (_topic_name_type_map[topic_name] == "sensor_msgs/msg/Imu")
        {
            // 反序列化
            auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
            rclcpp::Serialization<sensor_msgs::msg::Imu> serialization;
            serialization.deserialize_message(message.get(), imu_msg.get());
            imu_msg->header.stamp = now_time;
            // 序列化
            rclcpp::SerializedMessage serialized_msg;
            serialization.serialize_message(imu_msg.get(), &serialized_msg);
            _topic_publish_map[topic_name]->publish(serialized_msg);
        }
        else if (_topic_name_type_map[topic_name] == "tf2_msgs/msg/TFMessage")
        {
            // 反序列化
            auto tf_msg = std::make_shared<tf2_msgs::msg::TFMessage>();
            rclcpp::Serialization<tf2_msgs::msg::TFMessage> serialization;
            serialization.deserialize_message(message.get(), tf_msg.get());
            for (auto &transform : tf_msg->transforms)
            {
                transform.header.stamp = now_time;
            }
            // 序列化
            rclcpp::SerializedMessage serialized_msg;
            serialization.serialize_message(tf_msg.get(), &serialized_msg);
            _topic_publish_map[topic_name]->publish(serialized_msg);
        }
    }
    std::unordered_map<std::string, std::string> _topic_name_type_map;                         // 话题名称与类型映射
    std::unordered_map<std::string, rclcpp::GenericPublisher::SharedPtr> _topic_publish_map;   // 话题名称与发布者映射
    std::unordered_map<std::string, rclcpp::SubscriptionBase::SharedPtr> _topic_subscribe_map; // 话题名称与订阅者映射
};
RCLCPP_COMPONENTS_REGISTER_NODE(RosbagListener)