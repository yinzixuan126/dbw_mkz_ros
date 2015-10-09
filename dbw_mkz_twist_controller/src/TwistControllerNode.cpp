#include "TwistControllerNode.h"

namespace dbw_mkz_twist_controller{

TwistControllerNode::TwistControllerNode(ros::NodeHandle n, ros::NodeHandle pn)
{
  // Initialize
  loadParams(pn);
  pid_ = new PidControl;
  accel_pid_ = new PidControl;
  yaw_control_ = new YawControl;
  low_pass_ = new LowPass;
  fuel_level_ = 100.0;

  // Topics
  sub_twist_ = n.subscribe("cmd_vel", 1, &TwistControllerNode::recvTwist, this);
  sub_steering_ = n.subscribe("steering_report", 1, &TwistControllerNode::recvSteeringReport, this);
  sub_imu_ = n.subscribe("imu/data_raw", 1, &TwistControllerNode::recvImu, this);
  sub_enable_ = n.subscribe("dbw_enabled", 1, &TwistControllerNode::recvEnable, this);
  sub_fuel_level_ = n.subscribe("fuel_level_report", 1, &TwistControllerNode::recvFuel, this);

  pub_throttle_ = n.advertise<dbw_mkz_msgs::ThrottleCmd>("throttle_cmd", 1);
  pub_brake_ = n.advertise<dbw_mkz_msgs::BrakeCmd>("brake_cmd", 1);
  pub_steering_ = n.advertise<dbw_mkz_msgs::SteeringCmd>("steering_cmd", 1);

  pub_accel_ = n.advertise<std_msgs::Float64>("filtered_accel", 1);
  pub_req_accel_ = n.advertise<std_msgs::Float64>("req_accel", 1);

  // Timers
  control_timer_ = n.createTimer(ros::Duration(control_period_), &TwistControllerNode::controlCallback, this);

  // Dynamic reconfigure
  srv_.setCallback(boost::bind(&TwistControllerNode::reconfig, this, _1, _2));
}

void TwistControllerNode::controlCallback(const ros::TimerEvent& event)
{
  if ((event.current_real - cmd_stamp_).toSec() > (10.0 * control_period_)){
    pid_->resetIntegrator();
    accel_pid_->resetIntegrator();
    return;
  }

  double vehicle_mass = cfg_.vehicle_mass + fuel_level_ / 100.0 * cfg_.fuel_capacity * GAS_DENSITY;
  double vel_error = cmd_vel_.linear.x - actual_.linear.x;
  if ((fabs(cmd_vel_.linear.x) < mphToMps(1.0)) || !cfg_.pub_pedals){
    pid_->resetIntegrator();
  }

  double accel_cmd = pid_->step(vel_error, control_period_);

  if (cmd_vel_.linear.x < mphToMps(5.0)){
    accel_cmd = std::min(accel_cmd, -530 / vehicle_mass / cfg_.wheel_radius);
  }

  std_msgs::Float64 accel_cmd_msg;
  accel_cmd_msg.data = accel_cmd;
  pub_req_accel_.publish(accel_cmd_msg);

  if (sys_enable_){
    dbw_mkz_msgs::ThrottleCmd throttle_cmd;
    dbw_mkz_msgs::BrakeCmd brake_cmd;
    dbw_mkz_msgs::SteeringCmd steering_cmd;

    throttle_cmd.enable = true;
    if (accel_cmd >= 0){
      double rel_throttle = accel_pid_->step(accel_cmd - accel_, control_period_);
      throttle_cmd.pedal_cmd = throttleRelToActual(rel_throttle);
    }else{
      accel_pid_->resetIntegrator();
      throttle_cmd.pedal_cmd = 0;
    }

    brake_cmd.enable = true;
    brake_cmd.pedal_cmd_type = dbw_mkz_msgs::BrakeCmd::CMD_TORQUE;
    if ((accel_cmd < -cfg_.brake_deadband) || (cmd_vel_.linear.x < mphToMps(5.0))){
      brake_cmd.pedal_cmd = -accel_cmd * vehicle_mass * cfg_.wheel_radius;
    }else{
      brake_cmd.pedal_cmd = 0;
    }

    steering_cmd.enable = true;
    steering_cmd.steering_wheel_angle_cmd = yaw_control_->getSteeringWheel(cmd_vel_.angular.z, actual_.linear.x)
        + cfg_.steer_kp * (cmd_vel_.angular.z - actual_.angular.z);

    if (cfg_.pub_pedals){
      pub_throttle_.publish(throttle_cmd);
      pub_brake_.publish(brake_cmd);
    }

    if (cfg_.pub_steering){
      pub_steering_.publish(steering_cmd);
    }
  }else{
    pid_->resetIntegrator();
    accel_pid_->resetIntegrator();
  }
}

void TwistControllerNode::reconfig(ControllerConfig& config, uint32_t level)
{
  cfg_ = config;
  cfg_.vehicle_mass -= cfg_.fuel_capacity * GAS_DENSITY; // Subtract weight of full gas tank
  cfg_.vehicle_mass += 150.0; // Account for some passengers

  pid_->setGains(cfg_.speed_kp, 0.0, 0.0, -cfg_.decel_max, cfg_.accel_max);
  accel_pid_->setGains(cfg_.accel_kp, cfg_.accel_ki, 0.0, 0.0, 1.0);
  yaw_control_->setMaxLateralAccel(cfg_.max_lat_accel);
  low_pass_->setParams(cfg_.accel_tau, 0.02);
}

void TwistControllerNode::recvTwist(const geometry_msgs::Twist::ConstPtr& msg)
{
  cmd_vel_ = *msg;
  cmd_stamp_ = ros::Time::now();
}

void TwistControllerNode::recvFuel(const dbw_mkz_msgs::FuelLevelReport::ConstPtr& msg)
{
  fuel_level_ = msg->fuel_level;
}

void TwistControllerNode::loadParams(ros::NodeHandle pn)
{
  double control_rate;
  pn.param("control_rate", control_rate, 50.0);
  control_period_ = 1.0 / control_rate;
}

void TwistControllerNode::recvSteeringReport(const dbw_mkz_msgs::SteeringReport::ConstPtr& msg)
{
  double raw_accel = 50.0 * (msg->speed - actual_.linear.x);
  accel_ = low_pass_->filt(raw_accel);

  std_msgs::Float64 accel_msg;
  accel_msg.data = accel_;
  pub_accel_.publish(accel_msg);

  actual_.linear.x = msg->speed;
}

void TwistControllerNode::recvImu(const sensor_msgs::Imu::ConstPtr& msg)
{
  actual_.angular.z = msg->angular_velocity.z;
}

void TwistControllerNode::recvEnable(const std_msgs::Bool::ConstPtr& msg)
{
  sys_enable_ = msg->data;
}

}
