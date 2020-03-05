#include <cmath>
#include <iostream>
#include <string>
#include <algorithm>
#include <ros/ros.h>
#include <ros/message.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

class TwistMultiplier {
  public:
    TwistMultiplier(ros::NodeHandle &_nh, ros::NodeHandle &_nh_priv);
//    ~TwistMultiplier();

    void on_cmd_vel_in(const geometry_msgs::Twist::ConstPtr& msg);
    void on_speed_limit(const std_msgs::Float32::ConstPtr& msg);

  private:
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv;

    geometry_msgs::Twist last_twist;
    std_msgs::Float32 last_speed_limit;
    uint64_t last_speed_limit_time; // in ms

    std::string param_topic_cmd_vel_in;
    std::string param_topic_cmd_vel_out;
    std::string param_topic_speed_limit;
    bool param_scale_angular;
    int param_timeout;

    ros::Publisher pub_cmd_vel_out;
    ros::Subscriber sub_cmd_vel_in;
    ros::Subscriber sub_speed_limit;
};

TwistMultiplier::TwistMultiplier(ros::NodeHandle &_nh, ros::NodeHandle &_nh_priv):
  nh(_nh), nh_priv(_nh_priv) {
    ROS_INFO("initializing");

    nh_priv.param("topic_cmd_vel_in", param_topic_cmd_vel_in, (std::string)"cmd_vel_in");
    nh_priv.param("topic_cmd_vel_out", param_topic_cmd_vel_out, (std::string)"cmd_vel_out");
    nh_priv.param("topic_speed_limit", param_topic_speed_limit, (std::string)"speed_limit");
    nh_priv.param("scale_angular", param_scale_angular, (bool)true);
    nh_priv.param("timeout", param_timeout, (int)200); // if speed_limit is not received in [timeout] ms, spit out 0s

  pub_cmd_vel_out = nh.advertise<geometry_msgs::Twist>(param_topic_cmd_vel_out, 1);
  sub_cmd_vel_in = nh.subscribe<geometry_msgs::Twist>(param_topic_cmd_vel_in, 1, &TwistMultiplier::on_cmd_vel_in, this);
  sub_speed_limit = nh.subscribe<std_msgs::Float32>(param_topic_speed_limit, 1, &TwistMultiplier::on_speed_limit, this);
}

void TwistMultiplier::on_cmd_vel_in(const geometry_msgs::Twist::ConstPtr& msg) {
  last_twist = *msg;

  geometry_msgs::Twist msg_output;

  ros::Time time = ros::Time::now();
  uint64_t t = 1000 * (uint64_t)time.sec + (uint64_t)time.nsec / 1e6;

  if(param_timeout == 0 || (t - last_speed_limit_time < param_timeout)) {
    msg_output.linear.x = std::min<double>(std::max<double>(last_twist.linear.x, -last_speed_limit.data), last_speed_limit.data);
    msg_output.linear.y = std::min<double>(std::max<double>(last_twist.linear.y, -last_speed_limit.data), last_speed_limit.data);
    msg_output.linear.z = std::min<double>(std::max<double>(last_twist.linear.z, -last_speed_limit.data), last_speed_limit.data);

    if(param_scale_angular) {
      if(last_twist.linear.x != 0) {
        msg_output.angular.y *= msg_output.linear.x / last_twist.linear.x;
        msg_output.angular.z *= msg_output.linear.x / last_twist.linear.x;
      }
      if(last_twist.linear.y != 0) {
        msg_output.angular.x *= msg_output.linear.y / last_twist.linear.y;
        msg_output.angular.z *= msg_output.linear.y / last_twist.linear.y;
      }
      if(last_twist.linear.z != 0) {
        msg_output.angular.x *= msg_output.linear.z / last_twist.linear.z;
        msg_output.angular.y *= msg_output.linear.z / last_twist.linear.z;
      }
    }
  } else {
    // speed_limit timeout; send out 0's
    msg_output.linear.x = 0.;
    msg_output.linear.y = 0.;
    msg_output.linear.z = 0.;
    msg_output.angular.x = 0.;
    msg_output.angular.y = 0.;
    msg_output.angular.z = 0.;
  }

  pub_cmd_vel_out.publish(msg_output);
}

void TwistMultiplier::on_speed_limit(const std_msgs::Float32::ConstPtr& msg) {
  last_speed_limit = *msg;
  ros::Time time = ros::Time::now();
  last_speed_limit_time = 1000 * (uint64_t)time.sec + (uint64_t)time.nsec / 1e6;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "twist_speed_limit_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");
  TwistMultiplier node(nh, nh_priv);

  ros::Rate rate(100);

  while(ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }
}
