import os
import sys
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
sys.path.append(os.path.join(get_package_share_directory('rm_bringup'), 'launch'))


def generate_launch_description():

    from launch_ros.descriptions import ComposableNode
    from launch_ros.actions import ComposableNodeContainer, Node,SetParameter
    from launch.actions import TimerAction, Shutdown
    from launch import LaunchDescription

    launch_params = yaml.safe_load(open(os.path.join(
        get_package_share_directory('rm_bringup'), 'config', 'launch_params.yaml')))


    SetParameter(name='rune',value=launch_params['rune']),
    
    robot_description = Command(['xacro ', os.path.join(
        get_package_share_directory('rm_robot_description'), 'urdf', 'rm_robot.urdf.xacro'),
        ' xyz:=', launch_params['base2camera']['xyz'], ' rpy:=', launch_params['base2camera']['rpy']])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                    'publish_frequency': 1000.0}]
    )

    node_params = os.path.join(
        get_package_share_directory('rm_bringup'), 'config', 'node_params.yaml')

    # 图像
    # if launch_params['video_play']: 
    #     image_node  = ComposableNode(
    #         package='rm_camera_driver',
    #         plugin='fyt::camera_driver::VideoPlayerNode',
    #         name='video_player',
    #         parameters=[node_params],
    #         extra_arguments=[{'use_intra_process_comms': True}]
    #     )
    # else:
    #      image_node  = ComposableNode(
    #         package='rm_camera_driver',
    #         plugin='fyt::camera_driver::DahengCameraNode',
    #         name='camera_driver',
    #         parameters=[node_params],
    #         extra_arguments=[{'use_intra_process_comms': True}]
    #     )


    # 串口
    # if launch_params['virtual_serial']:
    #     serial_driver_node = Node(
    #         package='rm_serial_driver',
    #         executable='virtual_serial_node',
    #         name='virtual_serial',
    #         output='both',
    #         emulate_tty=True,
    #         parameters=[node_params],
    #         ros_arguments=['--ros-args', '--log-level',
    #                     'serial_driver:='+launch_params['serial_log_level']],
    #     )
    # else:
    #     serial_driver_node = Node(
    #         package='rm_serial_driver',
    #         executable='rm_serial_driver_node',
    #         name='serial_driver',
    #         output='both',
    #         emulate_tty=True,
    #         parameters=[node_params],
    #         ros_arguments=['--ros-args', '--log-level',
    #                     'serial_driver:='+launch_params['serial_log_level']],
    #     )
        
    # 装甲板识别
    # armor_detector_node = ComposableNode(
    #     package='armor_detector', 
    #     plugin='fyt::auto_aim::ArmorDetectorNode',
    #     name='armor_detector',
    #     parameters=[node_params],
    #     extra_arguments=[{'use_intra_process_comms': True}]
    # )
    
    # # 装甲板解算
    # if launch_params['hero_solver']:
    #     solver_node = Node(
    #         package='hero_armor_solver',
    #         executable='hero_armor_solver_node',
    #         name='armor_solver',
    #         output='both',
    #         emulate_tty=True,
    #         parameters=[node_params],
    #         ros_arguments=['--log-level', 'armor_solver:='+launch_params['solver_log_level']],
    #     )
    # else:
    #     solver_node = Node(
    #         package='armor_solver',
    #         executable='armor_solver_node',
    #         name='armor_solver',
    #         output='both',
    #         emulate_tty=True,
    #         parameters=[node_params],
    #         ros_arguments=['--log-level', 'armor_solver:='+launch_params['solver_log_level']],
    #     )

    # 打符
    # rune_detector_node = ComposableNode(    
    #     package='rune_detector',
    #     plugin='fyt::rune::RuneDetectorNode',
    #     name='rune_detector',
    #     parameters=[node_params],
    #     extra_arguments=[{'use_intra_process_comms': True}]
    #     )
    # rune_solver_node = Node(
    #     package='rune_solver',
    #     executable='rune_solver_node',
    #     name='rune_solver',
    #     output='both',
    #     emulate_tty=True,
    #     parameters=[node_params],
    #     arguments=['--ros-args','--log-level', 'rune_solver:='+launch_params['rune_solver_log_level']], 
    #     )

    # 使用intra cmmunication提高图像的传输速度
    # def get_camera_detector_container(*detector_nodes):
    #     nodes_list = list(detector_nodes)
    #     nodes_list.append(image_node)
    #     container = ComposableNodeContainer(
    #         name='camera_detector_container',
    #         namespace='',
    #         package='rclcpp_components',
    #         executable='component_container_mt',
    #         composable_node_descriptions=nodes_list,
    #         output='both',
    #         emulate_tty=True,
    #         ros_arguments=['--ros-args', '--log-level',
    #                        'armor_detector:='+launch_params['detector_log_level']],
    #     )
    #     return TimerAction(
    #         period=2.0,
    #         actions=[container],
    #     )

    # 延迟启动
    # delay_serial_node = TimerAction(
    #     period=1.5,
    #     actions=[serial_driver_node],
    # )

    # delay_solver_node = TimerAction(
    #     period=2.0,
    #     actions=[solver_node],
    # )
    
    # delay_rune_solver_node = TimerAction(
    #     period=2.0,
    #     actions=[rune_solver_node],
    # )
    # delay_cam_detector_node = TimerAction(
    #     period=2.0,
    #     actions=[],
    # )
    
    if launch_params['rune']:
        return LaunchDescription([
            robot_state_publisher,
            # delay_serial_node,
            # get_camera_detector_container(armor_detector_node, rune_detector_node),
            # delay_solver_node,
            # delay_rune_solver_node,
        ])
    else:
        return LaunchDescription([
            robot_state_publisher,
            # delay_serial_node,
            # get_camera_detector_container(armor_detector_node),
            # delay_solver_node
        ])
