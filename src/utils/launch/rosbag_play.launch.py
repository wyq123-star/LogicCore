'''
@Description: 用于播放rosbag2文件的launch文件
会自动读取开启文件夹下的ros bag文件
通过ros2bag与辅助节点来实现循环播放并时间戳一直往前
只会发布PointCloud2,Imu,tf三种类型的消息
@Author: Elaina
@Email:1463967532@qq.com
@Date: 2025-1-23 
'''
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import glob
import yaml
topic_names=[]
topic_types=[]
parameters={
    "rosbag_root":"./",
    "topic_suffix":"/bag"}
def generate_launch_description():
    rosbag_root=parameters["rosbag_root"]
    #转化成为绝对路径
    rosbag_root=os.path.abspath(rosbag_root)
    print(f"rosbag_root: {rosbag_root}")
    yaml_file_path=os.path.join(rosbag_root,"metadata.yaml")
    #查找绝对目录下后缀为.db3的文件
    db_file_path = glob.glob(os.path.join(rosbag_root, "*.db3"))
    if db_file_path:
        db_file_path = db_file_path[0]  # 取第一个找到的.db3文件
    else:
        raise ValueError("No .db3 file found in the specified directory.")
    getPlayYamlData(yaml_file_path)
    ld=LaunchDescription()
    #启动rosbag2播放器,开启循环模式
    #重定向
    topic_names_remap=[f"{topic_name}:={topic_name}{parameters['topic_suffix']}" for topic_name in topic_names]
    # topic_names_list=[f"{topic_name}{parameters['topic_suffix']}" for topic_name in topic_names]
    print(f"topic_names_remap: {topic_names_remap}")
    #开启rosbag cli工具
    rosbag_node_exe=ExecuteProcess(cmd=["ros2","bag","play","--loop",db_file_path,"--topics"]+topic_names
                                   +["--remap"]+topic_names_remap
                                   ,output="screen")
    #开启rosbag监听辅助节点
    rosbag_listener=Node(package="utils",
        executable="rosbag_listener_node",
        name="rosbag_listener_node",
        parameters=[{"topic_names":topic_names},{"topic_types":topic_types},{"topic_suffix":parameters["topic_suffix"]}],
        output="screen",
        emulate_tty=True
    )
    ld.add_action(rosbag_node_exe)
    ld.add_action(rosbag_listener)
    return ld

def getPlayYamlData(yaml_file_path:str):
    """_summary_
        这是一个获取rosbag2播放器的yaml文件的函数,会获得yaml文件中Pointcloud,imu,tf的topic名称和类型,
        并往全局变量topic_names和topic_types中添加

    Args:
        yaml_file_path (str): _description_ yaml文件的路径
    """
    with open(yaml_file_path, 'r') as file:
        yaml_node = yaml.safe_load(file)
    if "rosbag2_bagfile_information" in yaml_node:
        topics = yaml_node["rosbag2_bagfile_information"].get("topics_with_message_count", [])
        for topic in topics:
                topic_metadata = topic.get("topic_metadata", {})
                topic_type = topic_metadata.get("type", "")
                if topic_type == "sensor_msgs/msg/PointCloud2" or topic_type == "sensor_msgs/msg/Imu" or topic_type == "tf2_msgs/msg/TFMessage": 
                    topic_name = topic_metadata.get("name", "")
                    print(f"find topic: {topic_name} topic type: {topic_type}")
                    topic_types.append(topic_type)
                    topic_names.append(topic_name)            
                else:
                    continue
              