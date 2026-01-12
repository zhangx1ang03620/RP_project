from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # 正确的 MoveIt 配置加载
    moveit_config = (
        MoveItConfigsBuilder("panda", package_name="moveit_resources_panda_moveit_config")
        .to_dict()
    )

    # 启动 MTC Demo 节点
    pick_place_demo = Node(
        package="panda_pick_place",
        executable="mtc_tutorial",
        output="screen",
        parameters=[moveit_config],
    )

    return LaunchDescription([pick_place_demo])
