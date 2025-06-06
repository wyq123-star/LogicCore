#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import Point, PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

class EnhancedNavigationHandler:
    """增强版导航处理模块 - 支持动态目标点跟踪"""
    IDLE = 0          # 空闲状态，等待新目标
    NAVIGATING = 1    # 导航中状态
    RETRYING = 2      # 重试状态
    
    def __init__(self, node):
        self.node = node
        self.current_state = self.IDLE
        self.current_goal_handle = None
        self.goal_timeout = 60.0
        self.last_goal_time = 0.0
        self.failure_count = 0
        self.max_failures = 20  # 最大失败次数提高到20次
        self.active_goal = None  # 当前活跃目标点
        
        # 创建Action客户端连接官方导航
        self.nav_client = ActionClient(
            self.node, 
            NavigateToPose, 
            'navigate_to_pose'
        )
        
        # QoS配置
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        # 发布导航目标到官方话题
        self.goal_publisher = self.node.create_publisher(
            PoseStamped,
            '/goal_pose',
            qos_profile
        )
            
        # 订阅优化点话题
        self.optimal_sub = self.node.create_subscription(
            Point,
            '/optimal_point_data',
            self.optimal_point_callback,
            10
        )
        
        self.node.get_logger().info("🚀 导航处理器初始化完成，等待最优目标点...")
    
    def optimal_point_callback(self, msg):
        """处理优化点更新 - 仅在空闲状态保存并启动导航"""
        # 关键修改：仅在空闲状态处理新目标
        if self.current_state == self.IDLE:
            self.node.get_logger().info(f"📡 收到新优化点: x={msg.x:.2f}, y={msg.y:.2f}")
            self.start_navigation(msg)
        else:
            # 非空闲状态直接跳过，不保存目标点
            self.node.get_logger().debug("⏩ 当前非空闲状态，跳过新目标点")
    
    def start_navigation(self, point):
        """启动新导航任务"""
        self.active_goal = point
        self.failure_count = 0
        self.set_current_goal(point)
        self.publish_goal(point)
        self.current_state = self.NAVIGATING
    
    def publish_goal(self, point):
        """发布导航目标（已删除5秒间隔控制）"""
        # 构造PoseStamped消息
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = point.x
        goal_msg.pose.position.y = point.y
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.w = 1.0  # 默认朝向
        
        # 发布到官方导航话题
        self.goal_publisher.publish(goal_msg)
        self.node.get_logger().info(f"📍 发布目标: x={point.x:.2f}, y={point.y:.2f}")
        
        # 通过Action发送导航请求
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_msg
        
        # 确保Action服务器可用 - 添加超时机制
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error("🚨 导航服务器连接超时，跳过本次导航")
            self.reset_state()
            return
        
        # 发送目标并设置回调
        send_goal_future = self.nav_client.send_goal_async(
            nav_goal, 
            feedback_callback=self.nav_feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """处理目标响应 - 添加错误处理"""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.node.get_logger().warn("⚠️ 目标被导航服务器拒绝")
                self.handle_failure()
                return
                
            self.current_goal_handle = goal_handle
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.nav_result_callback)
            self.node.get_logger().info("🎯 目标已被导航服务器接受")
        except Exception as e:
            self.node.get_logger().error(f"🚨 目标响应处理异常: {str(e)}")
            self.handle_failure()
    
    def nav_feedback_callback(self, feedback_msg):
        """处理导航反馈（检查超时）"""
        current_time = time.time()
        # 添加反馈信息日志
        remaining_distance = feedback_msg.feedback.distance_remaining
        self.node.get_logger().info(f"📏 剩余距离: {remaining_distance:.2f}米")
        
        # 超时检查
        if current_time - self.last_goal_time > self.goal_timeout:
            self.node.get_logger().warn("⏰ 导航超时，取消当前任务")
            self.cancel_navigation()
    
    def nav_result_callback(self, future):
        """处理导航结果 - 重置状态"""
        try:
            result = future.result().result
            status = future.result().status
            
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.node.get_logger().info('✅ 导航成功')
            else:
                status_name = self.get_status_name(status)
                self.node.get_logger().warn(f'⚠️ 导航失败，状态: {status_name}')
                self.handle_failure()
            
            # 关键修改：仅重置状态，不处理缓存目标
            self.reset_state()
                
        except Exception as e:
            self.node.get_logger().error(f"🚨 导航结果处理异常: {str(e)}")
            self.handle_failure()
    
    def get_status_name(self, status):
        """获取状态码的文本描述"""
        status_map = {
            GoalStatus.STATUS_UNKNOWN: "未知",
            GoalStatus.STATUS_ACCEPTED: "已接受",
            GoalStatus.STATUS_EXECUTING: "执行中",
            GoalStatus.STATUS_CANCELING: "取消中",
            GoalStatus.STATUS_SUCCEEDED: "成功",
            GoalStatus.STATUS_CANCELED: "已取消",
            GoalStatus.STATUS_ABORTED: "已中止"
        }
        return status_map.get(status, "未知状态")
    
    def handle_failure(self):
        """统一处理导航失败情况"""
        self.failure_count += 1
        
        if self.failure_count < self.max_failures:
            self.node.get_logger().info(f'🔄 导航失败，当前连续失败次数: {self.failure_count}/{self.max_failures}')
            # 重新发布同一目标点（使用当前的active_goal）
            self.publish_goal(self.active_goal)
        else:
            self.node.get_logger().error(f'🚨 连续失败{self.max_failures}次，放弃当前目标')
            self.failure_count = 0
            self.reset_state()
    
    def cancel_navigation(self):
        """取消当前导航"""
        if self.current_goal_handle:
            future = self.current_goal_handle.cancel_goal_async()
            future.add_done_callback(self.cancel_done_callback)
    
    def cancel_done_callback(self, future):
        """取消操作完成回调"""
        try:
            response = future.result()
            if response.return_code == GoalStatus.STATUS_CANCELED:
                self.node.get_logger().info("🛑 导航已成功取消")
            else:
                self.node.get_logger().warn("⚠️ 取消失败")
            self.handle_failure()
        except Exception as e:
            self.node.get_logger().error(f"🚨 取消操作异常: {str(e)}")
            self.handle_failure()
    
    def reset_state(self):
        """重置状态为空闲 - 增强可靠性"""
        self.current_state = self.IDLE
        self.current_goal_handle = None
        self.failure_count = 0
        self.node.get_logger().info("🔄 导航状态已重置为空闲")
    
    def set_current_goal(self, goal):
        """设置当前目标点"""
        self.current_goal = goal
        self.last_goal_time = time.time()
        self.current_state = self.NAVIGATING
        self.node.get_logger().info(f"🎯 新目标已设置: x={goal.x:.2f}, y={goal.y:.2f}")

class OptimalGoalNavigator(Node):
    """最优目标导航节点"""
    def __init__(self):
        super().__init__('optimal_goal_navigator')
        self.navigation_handler = EnhancedNavigationHandler(self)
        self.get_logger().info("🚀 最优目标导航节点已启动")

def main(args=None):
    rclpy.init(args=args)
    node = OptimalGoalNavigator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 节点被手动终止")
        node.navigation_handler.cancel_navigation()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()