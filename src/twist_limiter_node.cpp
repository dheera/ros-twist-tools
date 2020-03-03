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

    void on_twist_input(const geometry_msgs::Twist::ConstPtr& msg);
    void on_multiplier(const std_msgs::Float32::ConstPtr& msg);

  private:
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv;

    geometry_msgs::Twist last_twist;
    std_msgs::Float32 last_multiplier;

    std::string param_topic_twist_input;
    std::string param_topic_twist_output;
    std::string param_topic_multiplier;

    ros::Publisher pub_twist_output;
    ros::Subscriber sub_twist_input;
    ros::Subscriber sub_multiplier;
};

TwistMultiplier::TwistMultiplier(ros::NodeHandle &_nh, ros::NodeHandle &_nh_priv):
  nh(_nh), nh_priv(_nh_priv) {
    ROS_INFO("initializing");

    nh_priv.param("topic_twist_input", param_topic_twist_input, (std::string)"/cmd_vel/managed_unsafe");
    nh_priv.param("topic_twist_output", param_topic_twist_output, (std::string)"/cmd_vel/managed");
    nh_priv.param("topic_twist_output", param_topic_multiplier, (std::string)"/cmd_vel/multiplier");

  pub_twist_output = nh.advertise<geometry_msgs::Twist>(param_topic_twist_output, 1);
  sub_twist_input = nh.subscribe<geometry_msgs::Twist>(param_topic_twist_input, 1, &TwistMultiplier::on_twist_input, this);
  sub_multiplier = nh.subscribe<std_msgs::Float32>(param_topic_multiplier, 1, &TwistMultiplier::on_multiplier, this);
}

void TwistMultiplier::on_twist_input(const geometry_msgs::Twist::ConstPtr& msg) {
  last_twist = *msg;

  geometry_msgs::Twist msg_output;
  msg_output.linear.x = last_twist.linear.x * last_multiplier.data;
  msg_output.angular.z = last_twist.angular.z * last_multiplier.data;

  ROS_INFO_STREAM("twist = " << last_twist.linear.x << " " << last_twist.linear.z);
  pub_twist_output.publish(msg_output);
}

void TwistMultiplier::on_multiplier(const std_msgs::Float32::ConstPtr& msg) {
  last_multiplier = *msg;
  ROS_INFO_STREAM("multiplier = " << last_multiplier.data);
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
