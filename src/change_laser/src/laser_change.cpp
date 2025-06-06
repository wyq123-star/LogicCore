#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class LaserTransformNode : public rclcpp::Node
{
public:
    LaserTransformNode()
        : Node("laser_transform_node")
    {
        // 1. 参数声明与初始化
        this->declare_parameter("input_laser_topic", "/laser/sd/raw");
        this->declare_parameter("output_topic", "/laser/sd/transformed");
        this->declare_parameter("target_frame", "odom");
        this->declare_parameter("laser_frame", "laser_link");
        update_parameters();

        // 2. 初始化TF2工具
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 3. 动态参数回调注册
        param_callback_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> &params)
            {
                return parameters_callback(params);
            });

        // 4. 创建发布/订阅
        recreate_subscription();
        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            output_topic_, rclcpp::SensorDataQoS());
    }

private:
    // 参数更新回调
    rcl_interfaces::msg::SetParametersResult parameters_callback(
        const std::vector<rclcpp::Parameter> &params)
    {
        std::lock_guard<std::mutex> lock(param_mutex_);
        auto result = rcl_interfaces::msg::SetParametersResult();
        result.successful = true;

        for (const auto &param : params)
        {
            const auto name = param.get_name();
            if (name == "input_laser_topic" && param.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
            {
                input_topic_ = param.as_string();
                recreate_subscription(); // 动态重建订阅
            }
            else if (name == "output_topic" && param.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
            {
                output_topic_ = param.as_string();
                publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
                    output_topic_, rclcpp::SensorDataQoS());
            }
            else if (name == "target_frame" && param.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
            {
                target_frame_ = param.as_string();
            }
            else if (name == "laser_frame" && param.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
            {
                laser_frame_ = param.as_string();
            }
        }
        return result;
    }

    // 动态重建订阅
    void recreate_subscription()
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            input_topic_, rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::LaserScan::SharedPtr msg)
            {
                laser_callback(msg);
            });
    }

    // 激光数据回调
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(param_mutex_);

        try
        {
            // 获取坐标变换
            auto transform = tf_buffer_->lookupTransform(
                target_frame_, laser_frame_, tf2::TimePointZero);

            // 创建新消息（实际变换逻辑需根据具体需求实现）
            auto transformed_msg = std::make_shared<sensor_msgs::msg::LaserScan>(*msg);
            transformed_msg->header.frame_id = target_frame_;

            // 应用变换（此处为示意，实际需实现坐标变换算法）
            apply_transform(transformed_msg, transform);

            publisher_->publish(*transformed_msg);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "TF转换失败: %s", ex.what());
        }
    }

    // 坐标变换应用（需根据实际需求完善）
    void apply_transform(
        sensor_msgs::msg::LaserScan::SharedPtr msg,
        const geometry_msgs::msg::TransformStamped &transform)
    {
        // 实现坐标变换逻辑
        // 示例：更新位姿信息（实际需实现点云坐标变换）
        msg->header.stamp = this->now();
        msg->header.frame_id = target_frame_;
    }

    // 更新参数值
    void update_parameters()
    {
        input_topic_ = this->get_parameter("input_laser_topic").as_string();
        output_topic_ = this->get_parameter("output_topic").as_string();
        target_frame_ = this->get_parameter("target_frame").as_string();
        laser_frame_ = this->get_parameter("laser_frame").as_string();
    }

    // 成员变量
    std::string input_topic_;
    std::string output_topic_;
    std::string target_frame_;
    std::string laser_frame_;

    std::mutex param_mutex_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaserTransformNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}