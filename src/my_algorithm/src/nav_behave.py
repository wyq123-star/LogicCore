#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import Point, PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter

class EnhancedNavigationHandler:
    """支持动态目标点跟踪和参数动态调整"""
    IDLE = 0          # 空闲状态，等待新目标
    NAVIGATING = 1    # 导航中状态
    RETRYING = 2      # 重试状态
    
    def __init__(self, node):
        self.node = node
        self.current_state = self.IDLE
        self.current_goal_handle = None
        self.last_goal_time = 0.0
        self.failure_count = 0
        self.active_goal = None
        self.pending_goal = None  # 等待中的目标点
        self.cancelling = False   # 取消操作标志

        # 声明动态参数（带默认值）
        self.node.declare_parameter('max_failures', 20)
        self.node.declare_parameter('goal_timeout', 60.0)
        self.node.declare_parameter('is_dynamic', True)  # 动态打断参数
        
        # 注册参数回调
        self.node.add_on_set_parameters_callback(self.parameters_callback)
        
        # 初始化参数值
        self.max_failures = self.node.get_parameter('max_failures').value
        self.goal_timeout = self.node.get_parameter('goal_timeout').value
        self.is_dynamic = self.node.get_parameter('is_dynamic').value  # 是否允许动态打断
        
        # 创建Action客户端
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

        # 发布导航目标
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
        
        self.node.get_logger().info(
            f"🚀 导航处理器初始化完成 | max_failures={self.max_failures} | goal_timeout={self.goal_timeout}s | is_dynamic={self.is_dynamic}"
        )
    
    def parameters_callback(self, params):
        """处理参数变化的回调函数"""
        result = SetParametersResult(successful=True)
        for param in params:
            if param.name == 'max_failures':
                self.max_failures = param.value
                self.node.get_logger().info(f"📌 更新 max_failures = {self.max_failures}")
            elif param.name == 'goal_timeout':
                self.goal_timeout = param.value
                self.node.get_logger().info(f"⏱️ 更新 goal_timeout = {self.goal_timeout}s")
            # 新增：动态打断参数处理
            elif param.name == 'is_dynamic':
                self.is_dynamic = param.value
                self.node.get_logger().info(f"🌀 更新 is_dynamic = {self.is_dynamic}")
                # 参数切换时清空等待中的目标
                if not self.is_dynamic and self.pending_goal:
                    self.node.get_logger().info("🛑 关闭动态模式，清空等待目标")
                    self.pending_goal = None
        return result

    def optimal_point_callback(self, msg):
        """处理优化点更新 - 新增动态打断功能"""
        # 动态模式且当前正在导航
        if self.is_dynamic and self.current_state == self.NAVIGATING:
            self.node.get_logger().info(f"🌀 收到动态目标: x={msg.x:.2f}, y={msg.y:.2f}")
            self.pending_goal = msg  # 保存新目标
            self.cancel_navigation()  # 取消当前导航
        # 空闲状态正常处理
        elif self.current_state == self.IDLE:
            self.node.get_logger().info(f"📡 收到新优化点: x={msg.x:.2f}, y={msg.y:.2f}")
            self.start_navigation(msg)
        # 非动态模式忽略新目标
        else:
            self.node.get_logger().debug("⏩ 当前非空闲状态，跳过新目标点")
    
    # 处理动态打断后的重启
    def handle_dynamic_restart(self):
        """处理动态打断后的重启逻辑"""
        if self.pending_goal:
            goal = self.pending_goal
            self.pending_goal = None
            self.node.get_logger().info("🔄 启动动态目标")
            self.start_navigation(goal)
        else:
            self.reset_state()
    
    def start_navigation(self, point):
        """启动新导航任务"""
        self.active_goal = point
        self.failure_count = 0
        self.set_current_goal(point)
        self.publish_goal(point)
        self.current_state = self.NAVIGATING
    
    def publish_goal(self, point):
        """发布导航目标"""
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = point.x
        goal_msg.pose.position.y = point.y
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.w = 1.0
        
        self.goal_publisher.publish(goal_msg)
        self.node.get_logger().info(f"📍 发布目标: x={point.x:.2f}, y={point.y:.2f}")
        
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_msg
        
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error("🚨 导航服务器连接超时")
            self.reset_state()
            return
        
        send_goal_future = self.nav_client.send_goal_async(
            nav_goal, 
            feedback_callback=self.nav_feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """处理目标响应"""
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
        remaining_distance = feedback_msg.feedback.distance_remaining
        self.node.get_logger().info(f"📏 剩余距离: {remaining_distance:.2f}米")
        
        if current_time - self.last_goal_time > self.goal_timeout:
            self.node.get_logger().warn("⏰ 导航超时，取消当前任务")
            self.cancel_navigation()
    
    def nav_result_callback(self, future):
        """处理导航结果 - 新增动态打断处理"""
        try:
            result = future.result().result
            status = future.result().status
            
            # 动态打断后的特殊状态处理
            if self.cancelling:
                self.cancelling = False
                self.handle_dynamic_restart()
                return
                
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.node.get_logger().info('✅ 导航成功')
            else:
                status_name = self.get_status_name(status)
                self.node.get_logger().warn(f'⚠️ 导航失败，状态: {status_name}')
                self.handle_failure()
            
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
            self.publish_goal(self.active_goal)
        else:
            self.node.get_logger().error(f'🚨 连续失败{self.max_failures}次，放弃当前目标')
            self.failure_count = 0
            self.reset_state()
    
    def cancel_navigation(self):
        """取消当前导航 - 新增动态打断标志"""
        if self.current_goal_handle:
            self.cancelling = True  # 设置取消标志
            self.node.get_logger().info("⏸️ 请求取消当前导航")
            future = self.current_goal_handle.cancel_goal_async()
            future.add_done_callback(self.cancel_done_callback)
    
    def cancel_done_callback(self, future):
        """取消操作完成回调 - 新增动态打断处理"""
        try:
            response = future.result()
            if response.return_code == GoalStatus.STATUS_CANCELED:
                self.node.get_logger().info("🛑 导航已成功取消")
            else:
                self.node.get_logger().warn("⚠️ 取消失败")
                self.cancelling = False  # 取消失败时重置标志
            
            # 不需要额外处理，nav_result_callback会处理后续
        except Exception as e:
            self.node.get_logger().error(f"🚨 取消操作异常: {str(e)}")
            self.cancelling = False  # 异常时重置标志
    
    def reset_state(self):
        """重置状态为空闲"""
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