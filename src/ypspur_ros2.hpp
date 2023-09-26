#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/header.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <ypspur_ros2/msg/control_mode.hpp>
#include <ypspur_ros2/msg/digital_output.hpp>
#include <ypspur_ros2/msg/digital_input.hpp>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <signal.h>
#include <string.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <sys/types.h>
#include <sys/wait.h>

#include <boost/atomic.hpp>
#include <boost/chrono.hpp>
#include <boost/thread.hpp>
#include <boost/thread/future.hpp>

#include <exception>
#include <map>
#include <string>
#include <vector>

namespace YP
{
#include <ypspur.h>
}

class YpspurRosNode : public rclcpp::Node
{
public: // private:
  std::thread thread_;
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> publisher_odom_;
  std::map<int, rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr> publishers_ads_;
  std::map<int, rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr> publishers_dios_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_cmd_vel_;
  std::map<int, rclcpp::Subscription<ypspur_ros2::msg::DigitalOutput>::SharedPtr> subscriber_dio_;
  std::shared_ptr<rclcpp::Publisher<ypspur_ros2::msg::DigitalInput>> publisher_digital_input_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  const tf2::Vector3 z_axis_;

  std::string port_;
  std::string param_file_;
  std::string ypspur_bin_;
  std::map<std::string, std::string> frames_;
  std::map<std::string, double> params_;
  int key_;
  bool simulate_;
  bool simulate_control_;

  double tf_time_offset_;

  pid_t pid_;

  enum OdometryMode
  {
    DIFF,
    NONE
  };
  OdometryMode mode_;

  class JointParams
  {
  public:
    int id_;
    std::string name_;
    double accel_;
    double vel_;
    double angle_ref_;
    double vel_ref_;
    double vel_end_;
    enum control_mode_
    {
      STOP,
      VELOCITY,
      POSITION,
      TRAJECTORY
    };
    control_mode_ control_;
    trajectory_msgs::msg::JointTrajectory cmd_joint_;
  };
  std::vector<JointParams> joints_;
  std::map<std::string, int> joint_name_to_num_;

  class AdParams
  {
  public:
    bool enable_;
    std::string name_;
    double gain_;
    double offset_;
  };
  class DioParams
  {
  public:
    bool enable_;
    std::string name_;
    bool input_;
    bool output_;
  };

  bool digital_input_enable_;
  std::vector<AdParams> ads_;
  std::vector<DioParams> dios_;
  const int ad_num_ = 8;
  unsigned int dio_output_;
  unsigned int dio_dir_;
  unsigned int dio_output_default_;
  unsigned int dio_dir_default_;
  const int dio_num_ = 8;
  std::map<int, rclcpp::Time> dio_revert_;

  int device_error_state_;
  int device_error_state_prev_;
  rclcpp::Time device_error_state_time_;

  geometry_msgs::msg::Twist::ConstSharedPtr cmd_vel_;
  rclcpp::Time cmd_vel_time_;
  rclcpp::Duration cmd_vel_expire_ = rclcpp::Duration(0, 0);

  int control_mode_;

  bool avoid_publishing_duplicated_odom_;
  bool publish_odom_tf_;
  rclcpp::Time previous_odom_stamp_;

  void cbControlMode(const ypspur_ros2::msg::ControlMode::ConstSharedPtr msg);

  void cbCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg);

  void cbDigitalOutput0(const ypspur_ros2::msg::DigitalOutput::SharedPtr msg);
  void cbDigitalOutput1(const ypspur_ros2::msg::DigitalOutput::SharedPtr msg);
  void cbDigitalOutput2(const ypspur_ros2::msg::DigitalOutput::SharedPtr msg);
  void cbDigitalOutput3(const ypspur_ros2::msg::DigitalOutput::SharedPtr msg);
  void cbDigitalOutput4(const ypspur_ros2::msg::DigitalOutput::SharedPtr msg);
  void cbDigitalOutput5(const ypspur_ros2::msg::DigitalOutput::SharedPtr msg);
  void cbDigitalOutput6(const ypspur_ros2::msg::DigitalOutput::SharedPtr msg);
  void cbDigitalOutput7(const ypspur_ros2::msg::DigitalOutput::SharedPtr msg);
  void cbDigitalOutput(const ypspur_ros2::msg::DigitalOutput::SharedPtr msg, int id_);

  void revertDigitalOutput(int id_);

  void updateDiagnostics(const rclcpp::Time& now, const bool connection_down = false);


public:
  YpspurRosNode();

  ~YpspurRosNode();

  void spinThreadFunction(std::shared_ptr<YpspurRosNode> &node);
};