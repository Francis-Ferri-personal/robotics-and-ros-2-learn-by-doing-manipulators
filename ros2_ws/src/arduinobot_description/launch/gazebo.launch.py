import os
from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    arduinobot_description_dir = get_package_share_directory("arduinobot_description")
    
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(
            arduinobot_description_dir,
            "urdf",
            "arduinobot.urdf.xacro",
        ),
        description="Aboslute path to the robot URDF file",
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value = [
            str(Path(arduinobot_description_dir).parent.resolve())
        ]
    )

    ros_distro = os.environ["ROS_DISTRO"]
    # This is only if you use ROS2 JAZZY
    physics_engine = "" if ros_distro == "humble" else "--physics-engine gz-physics-bullet-featherstone-plugin"
        

    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]))

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    robot_joint_publisher = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher'
        )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch"
            ), 
            "/gz_sim.launch.py"
            ]
        ),
        # We will add verbosity of 4 in gazebo for debugging errors
        launch_arguments=[
            ("gz_args", [" -v 4 -r empty.sdf ", physics_engine])
        ]
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description",
                   "-name", "arduinobot"]
    )

    # Create communication between ros messages and gazebo (gazeboo creates ros messages)
    # gz_ros2_bridge = Node(
    #     package="ros_gz_bridge",
    #     executable="parameter_bridge",
    #     arguments=[
    #         "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock]",
    #         '/camera@sensor_msgs/msg/Image@gz.msgs.Image',
    #         '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
    #     ]
    # )
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock]",
            "/image_raw@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo"
        ]
    )


    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            os.path.join(
                arduinobot_description_dir,
                "rviz",
                "display.rviz",
            ),
        ],
    )

    
    
    return LaunchDescription([
        model_arg,
        gazebo_resource_path,
        robot_state_publisher,
        robot_joint_publisher,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
        rviz_node
    ])

