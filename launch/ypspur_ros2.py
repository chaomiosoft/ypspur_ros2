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
        DeclareLaunchArgument('hz', default_value='200.0', description='Frequency Publish'),
        DeclareLaunchArgument('ad0_enable', default_value='false', description='A/D Input0 Enable'),
        DeclareLaunchArgument('ad1_enable', default_value='false', description='A/D Input1 Enable'),
        DeclareLaunchArgument('ad2_enable', default_value='false', description='A/D Input2 Enable'),
        DeclareLaunchArgument('ad3_enable', default_value='false', description='A/D Input3 Enable'),
        DeclareLaunchArgument('ad4_enable', default_value='false', description='A/D Input4 Enable'),
        DeclareLaunchArgument('ad5_enable', default_value='false', description='A/D Input5 Enable'),
        DeclareLaunchArgument('ad6_enable', default_value='false', description='A/D Input6 Enable'),
        DeclareLaunchArgument('ad7_enable', default_value='false', description='A/D Input7 Enable'),
        DeclareLaunchArgument('dio0_enable', default_value='false', description='DigitalInput0 Enable'),
        DeclareLaunchArgument('dio1_enable', default_value='false', description='DigitalInput1 Enable'),
        DeclareLaunchArgument('dio2_enable', default_value='false', description='DigitalInput2 Enable'),
        DeclareLaunchArgument('dio3_enable', default_value='false', description='DigitalInput3 Enable'),
        DeclareLaunchArgument('dio4_enable', default_value='false', description='DigitalInput4 Enable'),
        DeclareLaunchArgument('dio5_enable', default_value='false', description='DigitalInput5 Enable'),
        DeclareLaunchArgument('dio6_enable', default_value='false', description='DigitalInput6 Enable'),
        DeclareLaunchArgument('dio7_enable', default_value='false', description='DigitalInput7 Enable'),

        # ypspur_ros2 ノードの起動
        Node(
            package='ypspur_ros2',  # パッケージ名を指定
            executable='ypspur_bridge',     # 実行可能ファイル名を指定
            name='ypspur_ros2',           # ノード名を指定
            output='screen',              # 出力をターミナルに表示
            parameters=[                  # パラメータの設定
                {'param_file': LaunchConfiguration('param_file')},
                {'port': LaunchConfiguration('port')},
                {'simulate_control': LaunchConfiguration('simulate_control')},
                {'hz': LaunchConfiguration('hz')},
                {'ad7_enable': LaunchConfiguration('ad7_enable')},
                {'dio0_enable': LaunchConfiguration('dio0_enable')},
                {'dio1_enable': LaunchConfiguration('dio1_enable')},
                {'dio2_enable': LaunchConfiguration('dio2_enable')},
                {'dio3_enable': LaunchConfiguration('dio3_enable')},
                {'dio4_enable': LaunchConfiguration('dio4_enable')},
                {'dio5_enable': LaunchConfiguration('dio5_enable')},
                {'dio6_enable': LaunchConfiguration('dio6_enable')},
                {'dio7_enable': LaunchConfiguration('dio7_enable')},
            ]
        )
    ])
