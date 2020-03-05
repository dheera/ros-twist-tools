#include <cmath>
#include <iostream>
#include <string>
#include <ros/ros.h>
#include <ros/message.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

class TwistMultiplier {
  public:
    TwistMultiplier(ros::NodeHandle &_nh, ros::NodeHandle &_nh_priv);
//    ~TwistMultiplier();

    void on_cmd_vel_in(const geometry_msgs::Twist::ConstPtr& msg);
    void on_multiplier(const std_msgs::Float32::ConstPtr& msg);

  private:
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv;

    geometry_msgs::Twist last_twist;
    std_msgs::Float32 last_multiplier;
    uint64_t last_multiplier_time; // in ms

    std::string param_topic_in;
    std::string param_topic_out;
    std::string param_topic_multiplier;
    bool param_multiply_linear;
    bool param_multiply_angular;
    int param_timeout;

    ros::Publisher pub_cmd_vel_out;
    ros::Subscriber sub_cmd_vel_in;
    ros::Subscriber sub_multiplier;
};

TwistMultiplier::TwistMultiplier(ros::NodeHandle &_nh, ros::NodeHandle &_nh_priv):
  nh(_nh), nh_priv(_nh_priv) {
    ROS_INFO("initializing");

    nh_priv.param("topic_in", param_topic_in, (std::string)"cmd_vel_in");
    nh_priv.param("topic_out", param_topic_out, (std::string)"cmd_vel_out");
    nh_priv.param("topic_multiplier", param_topic_multiplier, (std::string)"multiplier");
    nh_priv.param("multiply_linear", param_multiply_linear, (bool)true);
    nh_priv.param("multiply_angular", param_multiply_angular, (bool)true);
    nh_priv.param("timeout", param_timeout, (int)200); // if multiplier is not received in [timeout] ms, spit out 0s

  pub_cmd_vel_out = nh.advertise<geometry_msgs::Twist>(param_topic_out, 1);
  sub_cmd_vel_in = nh.subscribe<geometry_msgs::Twist>(param_topic_in, 1, &TwistMultiplier::on_cmd_vel_in, this);
  sub_multiplier = nh.subscribe<std_msgs::Float32>(param_topic_multiplier, 1, &TwistMultiplier::on_multiplier, this);
}

void TwistMultiplier::on_cmd_vel_in(const geometry_msgs::Twist::ConstPtr& msg) {
  last_twist = *msg;

  geometry_msgs::Twist msg_output;

  ros::Time time = ros::Time::now();
  uint64_t t = 1000 * (uint64_t)time.sec + (uint64_t)time.nsec / 1e6;

  if(param_timeout == 0 || (t - last_multiplier_time < param_timeout)) {

    msg_output.linear.x = last_twist.linear.x;
    msg_output.linear.y = last_twist.linear.y;
    msg_output.linear.y = last_twist.linear.y;
    if(param_multiply_linear) {
      msg_output.linear.x *= last_multiplier.data;
      msg_output.linear.y *= last_multiplier.data;
      msg_output.linear.z *= last_multiplier.data;
    }

    msg_output.angular.x = last_twist.angular.x;
    msg_output.angular.y = last_twist.angular.y;
    msg_output.angular.z = last_twist.angular.z;
    if(param_multiply_angular) {
      msg_output.angular.x *= last_multiplier.data;
      msg_output.angular.y *= last_multiplier.data;
      msg_output.angular.z *= last_multiplier.data;
    }
  } else {
    // multiplier timeout; send out 0's
    msg_output.linear.x = 0.;
    msg_output.linear.y = 0.;
    msg_output.linear.z = 0.;
    msg_output.angular.x = 0.;
    msg_output.angular.y = 0.;
    msg_output.angular.z = 0.;
  }

  pub_cmd_vel_out.publish(msg_output);
}

void TwistMultiplier::on_multiplier(const std_msgs::Float32::ConstPtr& msg) {
  last_multiplier = *msg;
  ros::Time time = ros::Time::now();
  last_multiplier_time = 1000 * (uint64_t)time.sec + (uint64_t)time.nsec / 1e6;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "twist_multiplier_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");
  TwistMultiplier node(nh, nh_priv);

  ros::Rate rate(100);

  while(ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }
}
