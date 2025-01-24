/**
 * @file rosbag_player.cpp
 * @author Elaina (1463967532@qq.com)
 * @brief rosbag播放节点,读取ros2 bag文件并发布,可以选择同步或异步发布
 * @version 0.1
 * @date 2025-01-23
 *
 * @copyright Copyright (c) 2025
 *
 */
#include "fmt/core.h"
#include "yaml.h"
#include <csignal>
#include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <thread>
void on_exit([[maybe_unused]] int sig)
{
    RCUTILS_LOG_INFO("Exit by Ctrl+C");
    rclcpp::shutdown();
    exit(0);
}

class RosbagPlayer : public rclcpp::Node
{
public:
    RosbagPlayer(const rclcpp::NodeOptions &options)
        : Node("rosbag_player_node", options)
    {
        this->declare_parameter("rosbag_root", "./");
        this->declare_parameter("use_async", false);
        this->get_parameter("rosbag_root", _rosbag_root);

        // 查找当前目录下带.db3后缀的文件
        std::filesystem::path rosbag_root_abs = std::filesystem::absolute(_rosbag_root);
        fmt::print("rosbag_root_abs: {}\n", rosbag_root_abs.string());
        // 绝对路径+文件名
        auto yaml_file_path = rosbag_root_abs.string() + "/metadata.yaml";
        for (const auto &entry : std::filesystem::directory_iterator(rosbag_root_abs))
        {
            if (entry.path().extension() == ".db3")
            {
                _rosbag_file = entry.path().string();
                fmt::print("find rosbag file: {}\n", _rosbag_file);
                break;
            }
        }
        if (_rosbag_file.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "No rosbag file found in %s", _rosbag_root.c_str());
            return;
        }
        signal(SIGINT, on_exit);
        _reader.open(_rosbag_file);
        getPlayYamlData(yaml_file_path);
        if (get_parameter("use_async").as_bool())
        {
            _processing_thread = std::make_shared<std::thread>(&RosbagPlayer::playBagAsync, this);
        }
        else
        {
            _processing_thread = std::make_shared<std::thread>(&RosbagPlayer::play_bag, this);
        }
        // _processing_thread = std::make_shared<std::thread>(&RosbagPlayer::playBagAsync, this);
    }

    ~RosbagPlayer()
    {
        if (_processing_thread && _processing_thread->joinable())
        {
            _processing_thread->join();
        }
    }

private:
    /**
     * @brief 读取yaml文件,获取话题名称与类型
     *
     * @param yaml_file_path
     */
    void getPlayYamlData(std::string yaml_file_path)
    {
        // auto yaml_file = _rosbag_root + "metadata.yaml";
        YAML::Node yaml_node = YAML::LoadFile(yaml_file_path);
        if (yaml_node["rosbag2_bagfile_information"]["topics_with_message_count"])
        {

            const auto &topics = yaml_node["rosbag2_bagfile_information"]["topics_with_message_count"];
            for (auto topic : topics)
            {
                auto topic_type = topic["topic_metadata"]["type"].as<std::string>();
                if (topic_type == "sensor_msgs/msg/PointCloud2")
                {
                    auto pointcloud_topic_name = topic["topic_metadata"]["name"].as<std::string>();
                    fmt::print("find cloudpoint topic: {}\n", pointcloud_topic_name);
                    _pointcloud_publishers[pointcloud_topic_name] = this->create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud_topic_name, 10);
                }
                else if (topic_type == "sensor_msgs/msg/Imu")
                {
                    auto imu_topic_name = topic["topic_metadata"]["name"].as<std::string>();
                    fmt::print("find imu topic: {}\n", imu_topic_name);
                    _imu_publishers[imu_topic_name] = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_name, 10);
                }
                else if (topic_type == "tf2_msgs/msg/TFMessage")
                {
                    auto tf_topic_name = topic["topic_metadata"]["name"].as<std::string>();
                    fmt::print("find tf topic: {}\n", tf_topic_name);
                    _tf_publishers[tf_topic_name] = this->create_publisher<tf2_msgs::msg::TFMessage>(tf_topic_name, 10);
                }
                else
                {
                    continue;
                }
                _topic_names.push_back(topic["topic_metadata"]["name"].as<std::string>());
                // 范类型发布
                //  if (topic_type == "sensor_msgs/msg/PointCloud2" || topic_type == "sensor_msgs/msg/Imu" || topic_type == "tf2_msgs/msg/TFMessage")
                //  {
                //      auto topic_name = topic["topic_metadata"]["name"].as<std::string>();
                //      _topic_publish_map[topic_name] = this->create_generic_publisher(topic_name, topic_type, 10);
                //  }
            }
        }
    }
    /**
     * @brief 这里是以10hz的雷达作为参考,其他的话题序列化快不会触发延迟
     *
     */
    void play_bag()
    {
        while (rclcpp::ok())
        {
            if (!_reader.has_next())
            {
                _reader.open(_rosbag_file);
            }

            auto start_time = std::chrono::high_resolution_clock::now();
            auto bag_message = _reader.read_next();
            auto ros_time = rclcpp::Clock().now();
            auto topic_name = bag_message->topic_name;
            publishTopic(topic_name, bag_message);
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
            if ((duration < 100) && (duration > 1))
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(100) - std::chrono::milliseconds(duration));
            }
        }

        RCLCPP_INFO(this->get_logger(), "No more messages in the bag.");
        rclcpp::shutdown();
    }
    rclcpp::Time publishTopic(const std::string &topic_name, std::shared_ptr<rosbag2_storage::SerializedBagMessage> bag_message)
    {
        // auto bag_message = _reader.read_next();
        auto ros_time = rclcpp::Clock().now();
        // 返回阻塞信息;
        auto sleep_time = rclcpp::Time(0, 0);
        if (_pointcloud_publishers.count(topic_name) > 0)
        {
            auto pointcloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
            rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization;
            rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
            serialization.deserialize_message(&serialized_msg, pointcloud_msg.get());
            sleep_time = pointcloud_msg->header.stamp;
            pointcloud_msg->header.stamp = ros_time;
            _pointcloud_publishers[topic_name]->publish(*pointcloud_msg);
        }
        else if (_tf_publishers.count(topic_name) > 0)
        {
            auto tf_msg = std::make_shared<tf2_msgs::msg::TFMessage>();
            rclcpp::Serialization<tf2_msgs::msg::TFMessage> serialization;
            rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
            serialization.deserialize_message(&serialized_msg, tf_msg.get());
            // 赋值任意一个
            sleep_time = tf_msg->transforms[0].header.stamp;
            for (auto &tf : tf_msg->transforms)
            {
                tf.header.stamp = ros_time;
            }
            _tf_publishers[topic_name]->publish(*tf_msg);
        }
        else if (_imu_publishers.count(topic_name) > 0)
        {
            auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
            rclcpp::Serialization<sensor_msgs::msg::Imu> serialization;
            rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
            serialization.deserialize_message(&serialized_msg, imu_msg.get());
            sleep_time = imu_msg->header.stamp;
            imu_msg->header.stamp = ros_time;
            _imu_publishers[topic_name]->publish(*imu_msg);
        }
        // 范类型发布
        //  if (topic_type == "sensor_msgs/msg/PointCloud2" || topic_type == "sensor_msgs/msg/Imu" || topic_type == "tf2_msgs/msg/TFMessage")
        //  {
        //      auto topic_name = topic["topic_metadata"]["name"].as<std::string>();
        //      _topic_publish_map[topic_name] = this->create_generic_publisher(topic_name, topic_type, 10);
        //  }
        return sleep_time;
    }
    /**
     * @brief 异步发送,通过future来控制
     *
     */
    void playBagAsync()
    {
        while (rclcpp::ok())
        {
            if (!_reader.has_next())
            {
                _reader.open(_rosbag_file);
            }
            auto bag_message = _reader.read_next();
            auto topic_name = bag_message->topic_name;

            if (std::find(_topic_names.begin(), _topic_names.end(), topic_name) != _topic_names.end())
            {
                if (_topic_futures.count(topic_name) > 0)
                {
                    // fmt::print("time {} topic {}\n", _topic_sleep_time[topic_name].nanoseconds(), topic_name);
                    if (_topic_futures[topic_name]->valid())
                    {
                        // 同一个话题的future还在运行,等待
                        _topic_futures[topic_name]->wait();
                        // fmt::print("waitedtime: {} topic {}\n", std::chrono::high_resolution_clock::now().time_since_epoch().count(), topic_name);
                    }
                }
                auto start_time = std::chrono::high_resolution_clock::now();
                auto future = std::async(std::launch::async, [this, topic_name, bag_message, start_time]()
                                         {
                    //    auto start_time = std::chrono::high_resolution_clock::now();
                    auto time_stamp = publishTopic(topic_name, bag_message);
                    // fmt::print("publishtime:{}\n", std::chrono::high_resolution_clock::now().time_since_epoch().count());
                    if(_topic_sleep_time.count(topic_name)==0)
                    {
                        _topic_sleep_time[topic_name] = time_stamp;
                        // fmt::print("first time: {} topic {}\n", time_stamp.nanoseconds(),topic_name);
                    }
                    auto sleep_time = time_stamp-_topic_sleep_time[topic_name]; 
                    // fmt::print("sleep time: {}\n", sleep_time.nanoseconds());
                    _topic_sleep_time[topic_name] = time_stamp;
                    auto sleep_util_time=start_time+std::chrono::nanoseconds(sleep_time.nanoseconds());
                    //转化成为int64_t
                    // auto sleep_util_time_ns = std::chrono::time_point_cast<std::chrono::nanoseconds>(sleep_util_time).time_since_epoch().count();
                    // fmt::print("start time: {}\n" ,start_time.time_since_epoch().count());
                    // fmt::print("sleep time: {} topic {}\n", sleep_util_time_ns,topic_name);
                    if(sleep_time.nanoseconds()>0)
                    {
                        // std::this_thread::sleep_for(std::chrono::nanoseconds(sleep_time.nanoseconds()));
                        std::this_thread::sleep_until(sleep_util_time);
                    }
                    return true; });
                auto fut_ptr = std::make_shared<std::future<bool>>(std::move(future));
                _topic_futures[topic_name] = fut_ptr;
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            else
            {
                // std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }
        }
    }
    std::unordered_map<std::string, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> _pointcloud_publishers;
    std::unordered_map<std::string, rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr> _imu_publishers;
    std::unordered_map<std::string, rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr> _tf_publishers;
    rosbag2_cpp::Reader _reader;
    std::shared_ptr<std::thread> _processing_thread;
    std::string _rosbag_file;
    std::string _rosbag_root;
    // 存话题名称与对应publish的映射
    std::unordered_map<std::string, rclcpp::GenericPublisher::SharedPtr> _topic_publish_map;
    std::unordered_map<std::string, rclcpp::Time> _topic_sleep_time;
    std::unordered_map<std::string, std::shared_ptr<std::future<bool>>> _topic_futures;
    std::vector<std::string> _topic_names;
};

RCLCPP_COMPONENTS_REGISTER_NODE(RosbagPlayer)
