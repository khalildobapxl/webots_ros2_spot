#!/usr/bin/env python

import os
import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import Ros2SupervisorLauncher
from webots_ros2_driver.wait_for_controller_connection import (
    WaitForControllerConnection,
)


package_dir = get_package_share_directory("webots_spot")


# Define all the ROS 2 nodes that need to be restart on simulation reset here
def get_ros2_nodes(*args):
    # Manually launch the SpotArm driver to force localhost connection
    webots_controller_executable = os.path.join(get_package_share_directory('webots_ros2_driver'), 'scripts', 'webots-controller')
    spotarm_ros2_control_params = os.path.join(
        package_dir, "resource", "spotarm_ros2_controllers.yaml"
    )
    spotarm_driver = ExecuteProcess(
        cmd=[
            webots_controller_executable,
            '--robot-name=SpotArm',
            '--protocol=tcp',
            '--ip-address=127.0.0.1',  # Force localhost
            '--port=1234',
            'ros2',
            '--ros-args',
            '-p', 'robot_description:=' + os.path.join(package_dir, "resource", "spotarm_control.urdf"),
            '-p', 'use_sim_time:=True',
            '-p', 'set_robot_state_publisher:=False',
            '--params-file', spotarm_ros2_control_params,
        ],
        output='screen'
    )

    # ROS2 control spawners for SpotArm
    controller_manager_timeout = ["--controller-manager-timeout", "500"]
    controller_manager_prefix = "python.exe" if os.name == "nt" else ""
    trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        prefix=controller_manager_prefix,
        arguments=["spotarm_joint_trajectory_controller", "-c", "/controller_manager"]
        + controller_manager_timeout,
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        prefix=controller_manager_prefix,
        arguments=["spotarm_joint_state_broadcaster", "-c", "/controller_manager"]
        + controller_manager_timeout,
    )
    tiago_gripper_joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        prefix=controller_manager_prefix,
        arguments=[
            "tiago_gripper_joint_trajectory_controller",
            "-c",
            "/controller_manager",
        ]
        + controller_manager_timeout,
    )

    ros2_control_spawners = [
        trajectory_controller_spawner,
        joint_state_broadcaster_spawner,
        tiago_gripper_joint_trajectory_controller_spawner,
    ]

    # Wait for the simulation to be ready to start RViz, the navigation and spawners
    # We can't wait for the driver anymore, so we just return the spawners
    # waiting_nodes = WaitForControllerConnection(
    #     target_driver=spotarm_driver, nodes_to_start=ros2_control_spawners
    # )

    initial_manipulator_positioning = Node(
        package="webots_spot",
        executable="retract_manipulator",
        output="screen",
    )

    return [spotarm_driver, initial_manipulator_positioning] + ros2_control_spawners


def generate_launch_description():
    # Manually launch Webots to bypass OS detection issue in WebotsLauncher
    webots_executable = os.path.join(os.environ.get('WEBOTS_HOME', ''), 'webots')
    world_path = PathJoinSubstitution([package_dir, 'worlds', 'spot.wbt'])
    webots = ExecuteProcess(
        cmd=[webots_executable, world_path, '--batch', '--mode=realtime', '--stream', '--port=1234'],
        output='screen'
    )

    ros2_supervisor = Ros2SupervisorLauncher()

    # Manually launch the Spot driver to force localhost connection
    webots_controller_executable = os.path.join(get_package_share_directory('webots_ros2_driver'), 'scripts', 'webots-controller')
    spot_driver = ExecuteProcess(
        cmd=[
            webots_controller_executable,
            '--robot-name=Spot',
            '--protocol=tcp',
            '--ip-address=127.0.0.1',  # Force localhost
            '--port=1234',
            'ros2',
            '--ros-args',
            '-p', 'robot_description:=' + os.path.join(package_dir, "resource", "spot_control.urdf"),
            '-p', 'use_sim_time:=True',
            '-p', 'set_robot_state_publisher:=False'
        ],
        respawn=True,
        output='screen'
    )

    with open(os.path.join(package_dir, "resource", "spot.urdf")) as f:
        robot_desc = f.read()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_desc,
                "use_sim_time": True,
            }
        ],
    )

    # The following line is important!
    # This event handler respawns the ROS 2 nodes on simulation reset (supervisor process ends).
    reset_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=ros2_supervisor,
            on_exit=get_ros2_nodes,
        )
    )

    webots_event_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=webots,
            on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
        )
    )

    pointcloud_to_laserscan_node = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        remappings=[
            ("cloud_in", "/Spot/Velodyne_Puck/point_cloud"),
        ],
        parameters=[
            {
                "transform_tolerance": 0.01,
                "min_height": 0.0,
                "max_height": 1.0,
                "angle_min": -3.14,
                "angle_max": 3.14,
                "angle_increment": 0.00872,
                "scan_time": 0.1,
                "range_min": 0.9,
                "range_max": 100.0,
                "use_inf": True,
                "inf_epsilon": 1.0,
            }
        ],
        name="pointcloud_to_laserscan",
    )

    return LaunchDescription(
        [
            webots,
            ros2_supervisor,
            spot_driver,
            robot_state_publisher,
            webots_event_handler,
            reset_handler,
            pointcloud_to_laserscan_node,
        ]
        + get_ros2_nodes()
    )
