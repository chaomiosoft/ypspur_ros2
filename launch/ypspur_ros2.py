# my_launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # パラメータの設定
        DeclareLaunchArgument('param_file', default_value='../param/yp-spur.param', description='Parameter file path'),
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0', description='Serial port path'),

        # ypspur_ros2 ノードの起動
        Node(
            package='ypspur_ros2',  # パッケージ名を指定
            executable='ypspur_bridge',     # 実行可能ファイル名を指定
            name='ypspur_ros2',           # ノード名を指定
            output='screen',              # 出力をターミナルに表示
            parameters=[                  # パラメータの設定
                {'param_file': LaunchConfiguration('param_file')},
                {'port': LaunchConfiguration('port')}
            ]
        )
    ])
