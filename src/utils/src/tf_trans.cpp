/**
 * @file tf_trans.cpp
 * @author Elaina (1463967532@qq.com)
 * @brief 实现tf到变化矩阵的转换
 * @version 0.1
 * @date 2024-12-03
 *
 * @copyright Copyright (c) 2024
 */

#include "filesystem"
#include "fmt/core.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "fstream"
#include "yaml-cpp/yaml.h"
#include <map>
#include <string>
#include <vector>
#include <memory>

class TFTransNode_t : public rclcpp::Node {
public:
    TFTransNode_t(std::string name)
        : Node(name) {
        declare_parameter("tf_src", "base_link");   // 源tf
        declare_parameter("tf_dst", "camera_link"); // 目标tf
        _param_dictionary["tf_src"] = "";
        _param_dictionary["tf_dst"] = "";
        RCLCPP_INFO(this->get_logger(), "%s", fmt::format("node:{} is start", name).c_str());

        _tf_listener_ptr = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer_ptr, this);
        _timer_ptr = this->create_wall_timer(std::chrono::seconds(1), std::bind(&TFTransNode_t::timer_callback, this));
    }

    ~TFTransNode_t() {
        RCLCPP_INFO(this->get_logger(), "node is end");
    }

    void timer_callback() {
        if (_param_dictionary["tf_src"] != this->get_parameter("tf_src").as_string() || 
            _param_dictionary["tf_dst"] != this->get_parameter("tf_dst").as_string()) {
            _param_dictionary["tf_src"] = this->get_parameter("tf_src").as_string();
            _param_dictionary["tf_dst"] = this->get_parameter("tf_dst").as_string();
            RCLCPP_INFO(this->get_logger(), "%s", fmt::format("tf_src:{}", _param_dictionary["tf_src"]).c_str());
            RCLCPP_INFO(this->get_logger(), "%s", fmt::format("tf_dst:{}", _param_dictionary["tf_dst"]).c_str());
            saveTransForm2Yaml(tfTrans2Mat());
        }
    }

    tf2::Transform tfTrans2Mat() {
        try {
            auto tf_trans_stamp = _tf_buffer_ptr->lookupTransform(_param_dictionary["tf_dst"], _param_dictionary["tf_src"], tf2::TimePointZero, tf2::durationFromSec(2));
            tf2::Transform tf_trans;
            tf2::fromMsg(tf_trans_stamp.transform, tf_trans);

            tf2::Vector3 translation = tf_trans.getOrigin();
            tf2::Quaternion rotation = tf_trans.getRotation();
            tf2::Matrix3x3 rotation_matrix(rotation);

            fmt::print("变换矩阵为:\n");
            fmt::print("{: .3f} {: .3f} {: .3f} {: .3f}\n", rotation_matrix[0][0], rotation_matrix[0][1], rotation_matrix[0][2], translation.x());
            fmt::print("{: .3f} {: .3f} {: .3f} {: .3f}\n", rotation_matrix[1][0], rotation_matrix[1][1], rotation_matrix[1][2], translation.y());
            fmt::print("{: .3f} {: .3f} {: .3f} {: .3f}\n", rotation_matrix[2][0], rotation_matrix[2][1], rotation_matrix[2][2], translation.z());
            fmt::print("{: .3f} {: .3f} {: .3f} {: .3f}\n", 0.0, 0.0, 0.0, 1.0);

            double roll, pitch, yaw;
            rotation_matrix.getRPY(roll, pitch, yaw);
            fmt::print("欧拉角为: roll:{: .3f}, pitch:{: .3f}, yaw:{: .3f}\n", roll, pitch, yaw);
            return tf_trans;
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(get_logger(), "Transform error: %s", ex.what());
            return tf2::Transform();
        }
    }

    void saveTransForm2Yaml(tf2::Transform trans) {
        if (trans.getOrigin() == tf2::Vector3(0.0, 0.0, 0.0) && 
            trans.getRotation() == tf2::Quaternion(0.0, 0.0, 0.0, 1.0)) {
            RCLCPP_ERROR(get_logger(), "Transform is empty");
            return;
        }

        tf2::Vector3 translation = trans.getOrigin();
        tf2::Quaternion rotation = trans.getRotation();
        double raw, yaw, pitch;
        tf2::Matrix3x3 rotation_matrix(rotation);
        rotation_matrix.getEulerYPR(yaw, pitch, raw);

        YAML::Node transform_node;
        YAML::Node translation_node;
        translation_node["x"] = translation.x();
        translation_node["y"] = translation.y();
        translation_node["z"] = translation.z();
        transform_node["translation"] = translation_node;

        YAML::Node rotation_node;
        rotation_node["raw"] = raw;
        rotation_node["pitch"] = pitch;
        rotation_node["yaw"] = yaw;
        transform_node["rotation"] = rotation_node;

        double raw_data[16];
        trans.getOpenGLMatrix(raw_data);
        std::vector<double> vec(raw_data, raw_data + 16);
        transform_node["matrix"] = vec;

        try {
            std::ofstream fout(file_path.c_str());
            fout << transform_node;
            RCLCPP_INFO(this->get_logger(), "Transform saved to %s", file_path.c_str());
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to save transform: %s", e.what());
        }
    }

private:
    std::string file_path = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().parent_path().string() + "/config/output.yaml";
    std::shared_ptr<tf2_ros::Buffer> _tf_buffer_ptr = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    std::shared_ptr<tf2_ros::TransformListener> _tf_listener_ptr;
    std::shared_ptr<rclcpp::TimerBase> _timer_ptr;
    std::map<std::string, std::string> _param_dictionary;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TFTransNode_t>("tf_trans_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
