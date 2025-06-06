#include <geometry_msgs/msg/pose_array.hpp>
#include <iomanip>
#include <mutex>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

// 定义障碍物阈值
static constexpr int8_t OBSTACLE_THRESHOLD = 65; // 通常 >65 视为障碍物

class ObstacleExtractor : public rclcpp::Node
{
public:
    ObstacleExtractor()
        : Node("obstacle_extractor"), clear_count_(0)
    {
        // 1. 动态参数声明
        this->declare_parameter("costmap_topic", "/local_costmap/costmap");
        update_parameters();

        // 2. 初始化订阅器
        recreate_subscription();

        // 3. 障碍点发布器
        obstacle_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
            "/global_obstacles", 10);

        // 4. 注册参数回调
        param_callback_ = this->add_on_set_parameters_callback(
            std::bind(&ObstacleExtractor::param_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "节点已启动，监听话题: %s", costmap_topic_.c_str());
    }

private:
    void update_parameters()
    {
        costmap_topic_ = this->get_parameter("costmap_topic").as_string();
    }

    void recreate_subscription()
    {
        costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            costmap_topic_, 10,
            [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
            {
                costmap_callback(msg);
            });
    }

    void costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        geometry_msgs::msg::PoseArray obstacles;
        obstacles.header = msg->header; // 继承全局坐标系

        const auto &data = msg->data;
        const double res = msg->info.resolution;
        const double ox = msg->info.origin.position.x;
        const double oy = msg->info.origin.position.y;
        const uint32_t width = msg->info.width;
        const uint32_t height = msg->info.height;

        // 障碍物检测
        bool has_obstacles = false;
        for (uint32_t idx = 0; idx < data.size(); ++idx)
        {
            if (data[idx] > OBSTACLE_THRESHOLD && data[idx] != -1)
            {
                const uint32_t i = idx / width;
                const uint32_t j = idx % width;

                geometry_msgs::msg::Pose p;
                p.position.x = ox + (j + 0.5) * res;
                p.position.y = oy + (i + 0.5) * res;
                obstacles.poses.push_back(p);
                has_obstacles = true;
            }
        }

        // ==== 新增逻辑：障碍物消失计数 ====
        if (has_obstacles)
        {
            // 检测到障碍物，重置计数器
            clear_count_ = 0;
            obstacle_pub_->publish(obstacles);
            RCLCPP_INFO(this->get_logger(), "发布 %ld 个障碍点", obstacles.poses.size());
        }
        else
        {
            // 无障碍物，增加计数
            clear_count_++;

            if (clear_count_ >= 5)
            {
                // 连续5次无障碍物，发布空集合并重置计数器
                geometry_msgs::msg::PoseArray empty_obstacles;
                empty_obstacles.header.stamp = this->now();
                empty_obstacles.header.frame_id = msg->header.frame_id;
                obstacle_pub_->publish(empty_obstacles);
                RCLCPP_INFO(this->get_logger(), "连续5次未检测到障碍物，已清空障碍点");
                clear_count_ = 0; // 重置计数器
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "未检测到障碍物 (%d/5)", clear_count_);
            }
        }

        // 打印障碍点信息（只在有障碍物时显示）
        if (has_obstacles)
        {
            print_obstacle_info(obstacles, res, width, height);
        }
    }

    // 打印障碍点详细信息
    void print_obstacle_info(const geometry_msgs::msg::PoseArray &obstacles,
                             double resolution, uint32_t width, uint32_t height)
    {
        RCLCPP_INFO(this->get_logger(), "==============================");
        RCLCPP_INFO(this->get_logger(), "检测到 %ld 个障碍点", obstacles.poses.size());
        RCLCPP_INFO(this->get_logger(), "代价地图尺寸: %u x %u (%.2f m x %.2f m)",
                    width, height, width * resolution, height * resolution);
        RCLCPP_INFO(this->get_logger(), "分辨率: %.3f m", resolution);

        size_t print_count = std::min(static_cast<size_t>(5), obstacles.poses.size());
        for (size_t i = 0; i < print_count; ++i)
        {
            const auto &pose = obstacles.poses[i];
            RCLCPP_INFO(this->get_logger(), "障碍点 %zu: (%.2f, %.2f)",
                        i + 1, pose.position.x, pose.position.y);
        }

        if (obstacles.poses.size() > print_count)
        {
            RCLCPP_INFO(this->get_logger(), "... 还有 %ld 个障碍点未显示",
                        obstacles.poses.size() - print_count);
        }

        RCLCPP_INFO(this->get_logger(), "==============================");
    }

    // 动态参数回调
    rcl_interfaces::msg::SetParametersResult param_callback(
        const std::vector<rclcpp::Parameter> &params)
    {
        auto result = rcl_interfaces::msg::SetParametersResult();
        result.successful = true;

        for (const auto &param : params)
        {
            if (param.get_name() == "costmap_topic")
            {
                costmap_topic_ = param.as_string();
                recreate_subscription();
                RCLCPP_INFO(this->get_logger(), "切换话题至: %s", costmap_topic_.c_str());
            }
        }
        return result;
    }

    // 成员变量
    std::string costmap_topic_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr obstacle_pub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;
    int clear_count_; // 新增：障碍物消失计数器
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObstacleExtractor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}