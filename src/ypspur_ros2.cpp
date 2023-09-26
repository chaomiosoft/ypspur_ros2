#include "ypspur_ros2.hpp"

bool g_shutdown = false;
void sigintHandler(int sig)
{
  g_shutdown = true;
}

using namespace std::chrono_literals;

void YpspurRosNode::cbControlMode(const ypspur_ros2::msg::ControlMode::ConstSharedPtr msg)
{
  this->control_mode_ = msg->vehicle_control_mode;
  switch (this->control_mode_)
  {
    case ypspur_ros2::msg::ControlMode::OPEN:
      YP::YP_openfree();
      break;
    case ypspur_ros2::msg::ControlMode::TORQUE:
      YP::YPSpur_free();
      break;
    case ypspur_ros2::msg::ControlMode::VELOCITY:
      break;
  }
}

void YpspurRosNode::cbCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  this->cmd_vel_ = msg;
  this->cmd_vel_time_ = rclcpp::Clock(RCL_ROS_TIME).now();
  if (control_mode_ == ypspur_ros2::msg::ControlMode::VELOCITY)
  {
    YP::YPSpur_vel(msg->linear.x, msg->angular.z);
  }
}
void YpspurRosNode::cbDigitalOutput0(const ypspur_ros2::msg::DigitalOutput::SharedPtr msg)
{
  this->cbDigitalOutput(msg, 0);
}
void YpspurRosNode::cbDigitalOutput1(const ypspur_ros2::msg::DigitalOutput::SharedPtr msg)
{
  this->cbDigitalOutput(msg, 1);
}
void YpspurRosNode::cbDigitalOutput2(const ypspur_ros2::msg::DigitalOutput::SharedPtr msg)
{
  this->cbDigitalOutput(msg, 2);
}
void YpspurRosNode::cbDigitalOutput3(const ypspur_ros2::msg::DigitalOutput::SharedPtr msg)
{
  this->cbDigitalOutput(msg, 3);
}
void YpspurRosNode::cbDigitalOutput4(const ypspur_ros2::msg::DigitalOutput::SharedPtr msg)
{
  this->cbDigitalOutput(msg, 4);
}
void YpspurRosNode::cbDigitalOutput5(const ypspur_ros2::msg::DigitalOutput::SharedPtr msg)
{
  this->cbDigitalOutput(msg, 5);
}
void YpspurRosNode::cbDigitalOutput6(const ypspur_ros2::msg::DigitalOutput::SharedPtr msg)
{
  this->cbDigitalOutput(msg, 6);
}
void YpspurRosNode::cbDigitalOutput7(const ypspur_ros2::msg::DigitalOutput::SharedPtr msg)
{
  this->cbDigitalOutput(msg, 7);
}
void YpspurRosNode::cbDigitalOutput(const ypspur_ros2::msg::DigitalOutput::SharedPtr msg, int id_)
{
  const auto dio_output_prev = this->dio_output_;
  const auto dio_dir_prev = this->dio_dir_;
  const unsigned int mask = 1 << id_;

  switch (msg->output)
  {
  case ypspur_ros2::msg::DigitalOutput::HIGH_IMPEDANCE:
    this->dio_output_ &= ~mask;
    this->dio_dir_ &= ~mask;
    break;
  case ypspur_ros2::msg::DigitalOutput::LOW:
    this->dio_output_ &= ~mask;
    this->dio_dir_ |= mask;
    break;
  case ypspur_ros2::msg::DigitalOutput::HIGH:
    this->dio_output_ |= mask;
    this->dio_dir_ |= mask;
    break;
  case ypspur_ros2::msg::DigitalOutput::PULL_UP:
    this->dio_output_ |= mask;
    this->dio_dir_ &= ~mask;
    break;
  case ypspur_ros2::msg::DigitalOutput::PULL_DOWN:
    RCLCPP_ERROR(this->get_logger(), "Digital IO pull down is not supported on this system");
    break;
  }
  if (this->dio_output_ != dio_output_prev)
    YP::YP_set_io_data(this->dio_output_);
  if (this->dio_dir_ != dio_dir_prev)
    YP::YP_set_io_dir(this->dio_dir_);

  if (msg->toggle_time.sec > 0 || msg->toggle_time.nanosec > 0)
  {
    this->dio_revert_[id_] = rclcpp::Clock(RCL_ROS_TIME).now() + msg->toggle_time;
  }
}

void YpspurRosNode::revertDigitalOutput(int id_)
{
  const auto dio_output_prev = this->dio_output_;
  const auto dio_dir_prev = this->dio_dir_;
  const unsigned int mask = 1 << id_;

  this->dio_output_ &= ~mask;
  this->dio_output_ |= this->dio_output_default_ & mask;
  this->dio_dir_ &= ~mask;
  this->dio_dir_ |= this->dio_output_default_ & mask;

  if (this->dio_output_ != dio_output_prev)
    YP::YP_set_io_data(dio_output_);
  if (this->dio_dir_ != dio_dir_prev)
    YP::YP_set_io_dir(dio_dir_);

  this->dio_revert_[id_] = rclcpp::Time(0);
}

void YpspurRosNode::updateDiagnostics(const rclcpp::Time& now, const bool connection_down)
{
  const int connection_error = connection_down ? 1 : YP::YP_get_error_state();
  double t = 0;

  int err = 0;
  if (!connection_error)
    t = YP::YP_get_device_error_state(0, &err);
  this->device_error_state_ |= err;

  if (this->device_error_state_time_ + rclcpp::Duration(1.0) < now || connection_down ||
      this->device_error_state_ != this->device_error_state_prev_)
  {
    this->device_error_state_time_ = now;
    this->device_error_state_prev_ = this->device_error_state_;

    diagnostic_msgs::msg::DiagnosticArray msg;
    msg.header.stamp = now;
    msg.status.resize(1);
    msg.status[0].name = "YP-Spur Motor Controller";
    msg.status[0].hardware_id = "ipc-key" + std::to_string(this->key_);
    if (this->device_error_state_ == 0 && connection_error == 0)
    {
      if (t == 0)
      {
        msg.status[0].level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        msg.status[0].message = "Motor controller doesn't "
                                "provide device error state.";
      }
      else
      {
        if (rclcpp::Time(t, 0, RCL_ROS_TIME) < now - rclcpp::Duration(1.0))
        {
          msg.status[0].level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
          msg.status[0].message = "Motor controller doesn't "
                                  "update latest device error state.";
        }
        else
        {
          msg.status[0].level = diagnostic_msgs::msg::DiagnosticStatus::OK;
          msg.status[0].message = "Motor controller is running without error.";
        }
      }
    }
    else
    {
      msg.status[0].level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      if (connection_error)
        msg.status[0].message +=
            "Connection to ypspur-coordinator is down.";
      if (device_error_state_)
        msg.status[0].message +=
            std::string((msg.status[0].message.size() > 0 ? " " : "")) +
            "Motor controller reported error id " +
            std::to_string(this->device_error_state_) + ".";
    }
    msg.status[0].values.resize(2);
    msg.status[0].values[0].key = "connection_error";
    msg.status[0].values[0].value = std::to_string(connection_error);
    msg.status[0].values[1].key = "device_error";
    msg.status[0].values[1].value = std::to_string(this->device_error_state_);

    //pubs_["diag"].publish(msg);
    this->device_error_state_ = 0;
  }
}

YpspurRosNode::YpspurRosNode() : Node("ypspur_ros2")
  , tf_broadcaster_(this)
  , z_axis_(0, 0, 1)
{
  this->device_error_state_ = 0;
  this->device_error_state_prev_ = 0;
  this->device_error_state_time_ = rclcpp::Clock(RCL_ROS_TIME).now();
  this->avoid_publishing_duplicated_odom_ =  true;
  this->publish_odom_tf_ = true;
  this->previous_odom_stamp_ = rclcpp::Time(0L, RCL_ROS_TIME);

  this->declare_parameter<std::string>("port", "/dev/ttyACM0");
  this->get_parameter("port", this->port_);
  this->declare_parameter<int>("ipc_key", 28741);
  this->get_parameter("ipc_key", this->key_);
  this->declare_parameter<bool>("simulate", false);
  this->get_parameter("simulate", this->simulate_);
  this->declare_parameter<bool>("simulate_control", false);
  this->get_parameter("simulate_control", this->simulate_control_);
  if (this->simulate_control_)
    this->simulate_ = true;
  this->declare_parameter<std::string>("ypspur_bin", std::string("ypspur-coordinator"));
  this->get_parameter("ypspur_bin", this->ypspur_bin_);
  this->declare_parameter<std::string>("param_file", std::string(""));
  this->get_parameter("param_file", this->param_file_);
  this->declare_parameter<double>("tf_time_offset", 0.0);
  this->get_parameter("tf_time_offset", this->tf_time_offset_);

  double cmd_vel_expire_s;
  this->declare_parameter<double>("cmd_vel_expire", -1.0);
  this->get_parameter("cmd_vel_expire", cmd_vel_expire_s);
  this->cmd_vel_expire_ = rclcpp::Duration(cmd_vel_expire_s);

  std::string ad_mask("");
  this->ads_.resize(this->ad_num_);
  for (int i = 0; i < this->ad_num_; i++) 
  {
    const std::string key_name = std::string("ad") + std::to_string(i) + std::string("_enable");
    this->declare_parameter<bool>(key_name, false);
    this->get_parameter(key_name, ads_[i].enable_);
    const std::string key_name2 = std::string("ad") + std::to_string(i) + std::string("_name");
    this->declare_parameter<std::string>(key_name2, std::string("ad") + std::to_string(i));
    this->get_parameter(key_name2, this->ads_[i].name_);
    const std::string key_name3 = std::string("ad") + std::to_string(i) + std::string("_gain");
    this->declare_parameter<double>(key_name3, 1.0);
    this->get_parameter(key_name3, this->ads_[i].gain_);
    const std::string key_name4 = std::string("ad") + std::to_string(i) + std::string("_offset");
    this->declare_parameter<double>(key_name4, 0.0);
    this->get_parameter(key_name4, this->ads_[i].offset_);
    ad_mask = (ads_[i].enable_ ? std::string("1") : std::string("0")) + ad_mask;
    // create publish
    const std::string pub_name = std::string("ad/") + this->ads_[i].name_;
    auto publisher = this->create_publisher<std_msgs::msg::Float32>(pub_name, 1);
    this->publishers_ads_[i] = publisher;
  }
  this->digital_input_enable_ = false;
  this->dio_output_default_ = 0;
  this->dio_dir_default_ = 0;
  this->dios_.resize(this->dio_num_);
  for (int i = 0; i < this->dio_num_; i++) 
  {
    DioParams param;
    const std::string key_name1 = std::string("dio") + std::to_string(i) + std::string("_enable");
    this->declare_parameter<bool>(key_name1, false);
    this->get_parameter(key_name1, param.enable_);
    if (param.enable_)
    {
      const std::string key_name2 = std::string("dio") + std::to_string(i) + std::string("_name");
      this->declare_parameter<std::string>(key_name2, std::string(std::string("dio") + std::to_string(i)));
      this->get_parameter(key_name2, param.name_);
      const std::string key_name3 = std::string("dio") + std::to_string(i) + std::string("_output");
      this->declare_parameter<bool>(key_name3, true);
      this->get_parameter(key_name3, param.output_);
      const std::string key_name4 = std::string("dio") + std::to_string(i) + std::string("_input");
      this->declare_parameter<bool>(key_name4, false);
      this->get_parameter(key_name4, param.input_);
      
      if (param.output_)
      {
        if (i == 0)
          this->subscriber_dio_[i] = this->create_subscription<ypspur_ros2::msg::DigitalOutput>(
            param.name_, 1, std::bind(&YpspurRosNode::cbDigitalOutput0, this, std::placeholders::_1)
          );
        else if (i == 1)
          this->subscriber_dio_[i] = this->create_subscription<ypspur_ros2::msg::DigitalOutput>(
            param.name_, 1, std::bind(&YpspurRosNode::cbDigitalOutput1, this, std::placeholders::_1)
          );
        else if (i == 2)
          this->subscriber_dio_[i] = this->create_subscription<ypspur_ros2::msg::DigitalOutput>(
            param.name_, 1, std::bind(&YpspurRosNode::cbDigitalOutput2, this, std::placeholders::_1)
          );
        else if (i == 3)
          this->subscriber_dio_[i] = this->create_subscription<ypspur_ros2::msg::DigitalOutput>(
            param.name_, 1, std::bind(&YpspurRosNode::cbDigitalOutput3, this, std::placeholders::_1)
          );
        else if (i == 4)
          this->subscriber_dio_[i] = this->create_subscription<ypspur_ros2::msg::DigitalOutput>(
            param.name_, 1, std::bind(&YpspurRosNode::cbDigitalOutput4, this, std::placeholders::_1)
          );
        else if (i == 5)
          this->subscriber_dio_[i] = this->create_subscription<ypspur_ros2::msg::DigitalOutput>(
            param.name_, 1, std::bind(&YpspurRosNode::cbDigitalOutput5, this, std::placeholders::_1)
          );
        else if (i == 6)
          this->subscriber_dio_[i] = this->create_subscription<ypspur_ros2::msg::DigitalOutput>(
            param.name_, 1, std::bind(&YpspurRosNode::cbDigitalOutput6, this, std::placeholders::_1)
          );
        else if (i == 7)
          this->subscriber_dio_[i] = this->create_subscription<ypspur_ros2::msg::DigitalOutput>(
            param.name_, 1, std::bind(&YpspurRosNode::cbDigitalOutput7, this, std::placeholders::_1)
          );
      }

      std::string output_default;
      std::string default_key_name = std::string("dio") + std::to_string(i) + std::string("_default");
      this->declare_parameter<std::string>(default_key_name, std::string("HIGH_IMPEDANCE"));
      this->get_parameter(default_key_name, output_default);
      if (output_default.compare("HIGH_IMPEDANCE") == 0)
      {
      }
      else if (output_default.compare("LOW") == 0)
      {
        this->dio_dir_default_ |= 1 << i;
      }
      else if (output_default.compare("HIGH") == 0)
      {
        this->dio_dir_default_ |= 1 << i;
        this->dio_output_default_ |= 1 << i;
      }
      else if (output_default.compare("PULL_UP") == 0)
      {
        this->dio_output_default_ |= 1 << i;
      }
      else if (output_default.compare("PULL_DOWN") == 0)
      {
        RCLCPP_ERROR(this->get_logger(), "Digital IO pull down is not supported on this system");
      }
      if (param.input_)
        this->digital_input_enable_ = true;
    }
    dios_[i] = param;
  }
  this->dio_output_ = this->dio_output_default_;
  this->dio_dir_ = this->dio_dir_default_;
  if (this->digital_input_enable_)
  {
    // publish 
    this->publisher_digital_input_ = this->create_publisher<ypspur_ros2::msg::DigitalInput>("digital_input", 2);
  }

  this->declare_parameter<std::string>("odom_id", std::string("odom"));
  this->get_parameter("odom_id", this->frames_["odom"]);
  this->declare_parameter<std::string>("base_link_id", std::string("base_link"));
  this->get_parameter("base_link_id", this->frames_["base_link"]);
  this->declare_parameter<std::string>("origin_id", std::string(""));
  this->get_parameter("origin_id", this->frames_["origin"]);
  this->declare_parameter<double>("hz", 200.0);
  this->get_parameter("hz", this->params_["hz"]);

  std::string mode_name;
  this->declare_parameter<std::string>("OdometryMode", std::string("diff"));
  this->get_parameter("OdometryMode", mode_name);
  if (mode_name.compare("diff") == 0) {
    this->mode_ = DIFF;
    // publish wrench
    // publish odom 
    this->publisher_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 1);
    // subscribe cmd_vel
    this->subscriber_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 1, std::bind(&YpspurRosNode::cbCmdVel, this, std::placeholders::_1));
    
    this->declare_parameter<bool>("avoid_publishing_duplicated_odom", true);
    this->get_parameter("avoid_publishing_duplicated_odom", this->avoid_publishing_duplicated_odom_);
    this->declare_parameter<bool>("publish_odom_tf", true);
    this->get_parameter("publish_odom_tf", this->publish_odom_tf_);
  }
  else if (mode_name.compare("none") == 0)
  {

  }
  else
  {
    throw(std::runtime_error("unknown mode: " + mode_name));
  }

  // subscribe control_mode 
  this->control_mode_ = ypspur_ros2::msg::ControlMode::VELOCITY;

  // publish diag

  this->pid_ = 0;
  for (int i = 0; i < 2; i++) 
  {
    if (i > 0 || YP::YPSpur_initex(this->key_) < 0)
    {
      std::vector<std::string> args =
        {
          this->ypspur_bin_,
          "-d", this->port_,
          "--admask", ad_mask,
          "--msq-key", std::to_string(this->key_)
        };
      if (this->digital_input_enable_)
        args.push_back(std::string("--enable-get-digital-io"));
      if (this->simulate_)
        args.push_back(std::string("--without-device"));
      if (this->param_file_.size() > 0)
      {
        args.push_back(std::string("-p"));
        args.push_back(this->param_file_);
      }

      char** argv = new char*[args.size() + 1];
      for (unsigned int i = 0; i < args.size(); i++) 
      {
        argv[i] = new char[args[i].size() + 1];
        memcpy(argv[i], args[i].c_str(), args[i].size());
        argv[i][args[i].size()] = 0;
      }
      argv[args.size()] = nullptr;

      int msq = msgget(this->key_, 0666 | IPC_CREAT);
      msgctl(msq, IPC_RMID, nullptr);

      RCLCPP_WARN(this->get_logger(), "launching ypspur-coordinator");
      this->pid_ = fork();
      if (this->pid_ == -1)
      {
        const int err = errno;
        throw(std::runtime_error(std::string("failed to fork process: ") + strerror(err)));
      }
      else if (this->pid_ == 0)
      {
        execvp(this->ypspur_bin_.c_str(), argv);
        throw(std::runtime_error("failed to start ypspur-coordinator"));
      }

      for (unsigned int i = 0; i < args.size(); i++)
      {
        delete argv[i];
      }
      delete argv;

      for (int i = 4; i >= 0; i--)
      {
        int status;
        if (waitpid(this->pid_, &status, WNOHANG) == this->pid_)
        {
          if (WIFSIGNALED(status))
          {
            throw(std::runtime_error("ypspur-coordinator dead immediately by signal " + std::to_string(WTERMSIG(status))));
          }
          if (WIFEXITED(status))
          {
            throw(std::runtime_error("ypspur-coordinator dead immediately with exit code " + std::to_string(WEXITSTATUS(status))));
          }
          throw(std::runtime_error("ypspur-coordinator dead immediately"));
        }
        else if (i == 0)
        {
          throw(std::runtime_error("failed to init libypspur"));
        }
        rclcpp::sleep_for(1s);
        if (YP::YPSpur_initex(this->key_) >= 0)
          break;
      }
    }
    double ret;
    boost::atomic<bool> done(false);
    auto get_vel_thread = [&ret, &done]
    {
      double test_v, test_w;
      ret = YP::YPSpur_get_vel(&test_v, &test_w);
      done = true;
    };
    boost::thread spur_test = boost::thread(get_vel_thread);
    rclcpp::sleep_for(100ms);
    if (!done)
    {
      spur_test.detach();
      RCLCPP_WARN(this->get_logger(), "ypspur-coordinator seems to be down - launching");
      continue;
    }
    spur_test.join();
    if (ret < 0)
    {
      RCLCPP_WARN(this->get_logger(), "ypspur-coordinator returns error - launching");
      continue;
    }
    RCLCPP_WARN(this->get_logger(), "ypspur-coordinator launched");
    break;
  }

  RCLCPP_INFO(this->get_logger(), "ypspur-coordinator connected");
  signal(SIGINT, sigintHandler);

  YP::YP_get_parameter(YP::YP_PARAM_MAX_VEL, &this->params_["vel"]);
  YP::YP_get_parameter(YP::YP_PARAM_MAX_ACC_V, &this->params_["acc"]);
  YP::YP_get_parameter(YP::YP_PARAM_MAX_W, &this->params_["angvel"]);
  YP::YP_get_parameter(YP::YP_PARAM_MAX_ACC_W, &this->params_["angacc"]);

  if (!this->has_parameter("vel"))
    RCLCPP_WARN(this->get_logger(), "default \"vel\" %0.3f used", (float)this->params_["vel"]);
  if (!this->has_parameter("acc"))
    RCLCPP_WARN(this->get_logger(), "default \"acc\" %0.3f used", (float)this->params_["acc"]);
  if (!this->has_parameter("angvel"))
    RCLCPP_WARN(this->get_logger(), "default \"angvel\" %0.3f used", (float)this->params_["angvel"]);
  if (!this->has_parameter("angacc"))
    RCLCPP_WARN(this->get_logger(), "default \"angacc\" %0.3f used", (float)this->params_["angacc"]);

  this->declare_parameter<double>("vel", this->params_["vel"]);
  this->get_parameter("vel", this->params_["vel"]);
  this->declare_parameter<double>("acc", this->params_["acc"]);
  this->get_parameter("acc", this->params_["acc"]);
  this->declare_parameter<double>("angvel", this->params_["angvel"]);
  this->get_parameter("angvel", this->params_["angvel"]);
  this->declare_parameter<double>("angacc", this->params_["angacc"]);
  this->get_parameter("angacc", this->params_["angacc"]);

  YP::YPSpur_set_vel(this->params_["vel"]);
  YP::YPSpur_set_accel(this->params_["acc"]);
  YP::YPSpur_set_angvel(this->params_["angvel"]);
  YP::YPSpur_set_angaccel(this->params_["angacc"]);

  YP::YP_set_io_data(this->dio_output_);
  YP::YP_set_io_dir(this->dio_dir_);
}

YpspurRosNode::~YpspurRosNode()
{
  if (this->pid_ > 0 && YP::YP_get_error_state() == 0)
  {
    RCLCPP_INFO(this->get_logger(), "killing ypspur-coordinator (%d)", this->pid_);
    kill(this->pid_, SIGINT);
    int status;
    waitpid(this->pid_, &status, 0);
    RCLCPP_INFO(this->get_logger(), "ypspur-coordinator is killed (status: %d)", status);
  }
}

void YpspurRosNode::spinThreadFunction(std::shared_ptr<YpspurRosNode> &node)
{
  while (rclcpp::ok()) {
    geometry_msgs::msg::TransformStamped odom_trans;
    odom_trans.header.frame_id = this->frames_["odom"];
    odom_trans.child_frame_id = this->frames_["base_link"];

    nav_msgs::msg::Odometry odom;
    geometry_msgs::msg::WrenchStamped wrench;
    odom.header.frame_id = this->frames_["odom"];
    odom.child_frame_id = this->frames_["base_link"];
    wrench.header.frame_id = this->frames_["base_link"];

    odom.pose.pose.position.x = 0;
    odom.pose.pose.position.y = 0;
    odom.pose.pose.position.z = 0;
    odom.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(z_axis_, 0));
    odom.twist.twist.linear.x = 0;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = 0;

    std::map<int, geometry_msgs::msg::TransformStamped> joint_trans;
    sensor_msgs::msg::JointState joint;
    if (this->joints_.size() > 0)
    {
      // 未実装
    }

    RCLCPP_INFO(this->get_logger(), "ypspur_ros2 main loop started");
    // ros::Rate loop(params_["hz"]);
    rclcpp::Rate loop(this->params_["hz"]);
    while (!g_shutdown)
    {
      const auto now = rclcpp::Clock(RCL_ROS_TIME).now();
      const float dt = 1.0 / this->params_["hz"];

      if (this->cmd_vel_ && this->cmd_vel_expire_ > rclcpp::Duration(0))
      {
        if (this->cmd_vel_time_ + this->cmd_vel_expire_ < now)
        {
          // cmd_vel is too old and expired
          this->cmd_vel_ = nullptr;
          if (this->control_mode_ == ypspur_ros2::msg::ControlMode::VELOCITY)
            YP::YPSpur_vel(0.0, 0.0);
        }
      }

      if (this->mode_ == DIFF)
      {
        double x, y, yaw, v(0), w(0);
        double t;

        if (!this->simulate_control_)
        {
          t = YP::YPSpur_get_pos(YP::CS_BS, &x, &y, &yaw);
          if (t <= 0.0)
            break;
          YP::YPSpur_get_vel(&v, &w);
        }
        else
        {
          t = rclcpp::Clock(RCL_ROS_TIME).now().seconds();
          if (this->cmd_vel_)
          {
            v = this->cmd_vel_->linear.x;
            w = this->cmd_vel_->angular.z;
          }
          yaw = tf2::getYaw(odom.pose.pose.orientation) + dt * w;
          x = odom.pose.pose.position.x + dt * v * cosf(yaw);
          y = odom.pose.pose.position.y + dt * v * sinf(yaw);
        }

        const rclcpp::Time current_stamp(t, 0, RCL_ROS_TIME);
        if (!this->avoid_publishing_duplicated_odom_ || (current_stamp > this->previous_odom_stamp_))
        {
          odom.header.stamp = current_stamp;
          odom.pose.pose.position.x = x;
          odom.pose.pose.position.y = y;
          odom.pose.pose.position.z = 0;
          odom.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(z_axis_, yaw));
          odom.twist.twist.linear.x = v;
          odom.twist.twist.linear.y = 0;
          odom.twist.twist.angular.z = w;
          //pubs_["odom"].publish(odom);
          this->publisher_odom_->publish(odom);
          
          if (this->publish_odom_tf_)
          {
            odom_trans.header.stamp = current_stamp + rclcpp::Duration(tf_time_offset_);
            odom_trans.transform.translation.x = x;
            odom_trans.transform.translation.y = y;
            odom_trans.transform.translation.z = 0;
            odom_trans.transform.rotation = odom.pose.pose.orientation;
            //tf_broadcaster_.sendTransform(odom_trans);
            this->tf_broadcaster_.sendTransform(odom_trans);
          }
        }
        this->previous_odom_stamp_ = current_stamp;

        if (!this->simulate_control_)
        {
          t = YP::YPSpur_get_force(&wrench.wrench.force.x, &wrench.wrench.torque.z);
          if (t <= 0.0)
            break;
        }
        wrench.header.stamp = rclcpp::Time(t, 0, RCL_ROS_TIME);
        wrench.wrench.force.y = 0;
        wrench.wrench.force.z = 0;
        wrench.wrench.torque.x = 0;
        wrench.wrench.torque.y = 0;
        //pubs_["wrench"].publish(wrench);
        // ここは未実装とする

        if (this->frames_["origin"].length() > 0)
        {
          // 未実装
        }
      }
      if (this->joints_.size() > 0)
      {
        // 未実装
      }
      for (int i = 0; i < this->ad_num_; i++) 
      {
        if (this->ads_[i].enable_)
        {
          std_msgs::msg::Float32 ad;
          ad.data = YP::YP_get_ad_value(i) * ads_[i].gain_ + ads_[i].offset_;
          //pubs_["ad/" + ads_[i].name_].publish(ad);
          this->publishers_ads_[i]->publish(ad);
        }
      }

      if (this->digital_input_enable_)
      {
        ypspur_ros2::msg::DigitalInput din;

        din.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
        int in = YP::YP_get_ad_value(15);
        for (int i = 0; i < this->dio_num_; i++)
        {
          if (!this->dios_[i].enable_)
            continue;
          din.name.push_back(this->dios_[i].name_);
          if (in & (1 << i))
            din.state.push_back(true);
          else
            din.state.push_back(false);
        }
        this->publisher_digital_input_->publish(din);
      }

      for (int i = 0; i < this->dio_num_; i++)
      {
        if (this->dio_revert_[i] != rclcpp::Time(0))
        {
          if (this->dio_revert_[i] < now)
          {
            this->revertDigitalOutput(i);
          }
        }
      }
      this->updateDiagnostics(rclcpp::Clock(RCL_ROS_TIME).now());

      if (YP::YP_get_error_state())
        break;

      rclcpp::spin_some(node);
      loop.sleep();

      int status;
      if (waitpid(pid_, &status, WNOHANG) == pid_)
      {
        if (WIFEXITED(status))
        {
          RCLCPP_ERROR(this->get_logger(), "ypspur-coordinator exited");
        }
        else
        {
          if (WIFSTOPPED(status))
          {
            RCLCPP_ERROR(this->get_logger(), "ypspur-coordinator dead with signal %d",
                      WSTOPSIG(status));
          }
          else
          {
            RCLCPP_ERROR(this->get_logger(), "ypspur-coordinator dead");
          }
          this->updateDiagnostics(rclcpp::Clock(RCL_ROS_TIME).now(), true);
        }
        break;
      }
    }
    RCLCPP_INFO(this->get_logger(), "ypspur_ros main loop terminated");

    if (YP::YP_get_error_state())
    {
      RCLCPP_ERROR(this->get_logger(), "ypspur-coordinator is not active");
      return;
    }
    return;
  }
}

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);

  printf("hello world ypspur_ros2 package\n");

  auto node = std::make_shared<YpspurRosNode>();

  RCLCPP_INFO(node->get_logger(), "Hello, ROS2 world!");

  // スレッド開始
  node->spinThreadFunction(node);

  RCLCPP_INFO(node->get_logger(), "rclcpp::spin(node)");
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}