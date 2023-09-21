# my_launch.py

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    ypspur_ros2_ypspur_param_dir = os.path.dirname(os.path.realpath(__file__)) + '/../param/yp-spur.param'
    print(ypspur_ros2_ypspur_param_dir)
    return LaunchDescription([
        # パラメータの設定
        DeclareLaunchArgument('simulate_control', default_value='true', description='Simulate Device'),
        DeclareLaunchArgument('param_file', default_value=ypspur_ros2_ypspur_param_dir, description='Parameter file path'),
        DeclareLaunchArgument('port', default_value='/tmp/ttyACM0', description='Serial port path'),

        # ypspur_ros2 ノードの起動
        Node(
            package='ypspur_ros2',  # パッケージ名を指定
            executable='ypspur_bridge',     # 実行可能ファイル名を指定
            name='ypspur_ros2',           # ノード名を指定
            output='screen',              # 出力をターミナルに表示
            parameters=[                  # パラメータの設定
                {'param_file': LaunchConfiguration('param_file')},
                {'port': LaunchConfiguration('port')},
                {'simulate_control': LaunchConfiguration('simulate_control')}
            ]
        )
    ])
