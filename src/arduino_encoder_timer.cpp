
/*
 * This program converts the raw encoder values from the Arduino into a message consisting of
 * dt, and change in encoder readings in dt time
 */

#include <ros/ros.h>
#include <ros/console.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int64.h>
#include <agv/EncoderCount.h>
#include <cmath>

class ArduinoEncoderTimer
{
public:
  ArduinoEncoderTimer();

private:
  ros::NodeHandle nh;
  ros::Subscriber encoder_raw_sub_left;
  ros::Subscriber encoder_raw_sub_right;
  ros::Publisher encoder_count_pub;
  ros::Publisher arduino_timer_pub;
  ros::Timer encoder_timer;

  agv::EncoderCount encoder_msg;
  ros::Time previous_time;
  std_msgs::Bool timer_msg;

  long left_count;
  long right_count;
  long prev_left_count = 0;
  long prev_right_count = 0;
  bool left_count_updated;
  bool right_count_updated;

  void rawLeftEncoderCallback(const std_msgs::Int64::ConstPtr& raw_left_encoder_msg);
  void rawRightEncoderCallback(const std_msgs::Int64::ConstPtr& raw_right_encoder_msg);
  void mainTimerCallback(const ros::TimerEvent& timer_event);
  void publish_to_encoder_topic();
};

ArduinoEncoderTimer::ArduinoEncoderTimer()
{
  ros::NodeHandle private_nh("~");

  std::string raw_left_encoder_count_topic;
  std::string raw_right_encoder_count_topic;
  std::string encoder_timer_topic;
  std::string encoder_count_publish_topic;
  double timer_period;

  ROS_ASSERT(private_nh.getParam("raw_left_encoder_count_topic", raw_left_encoder_count_topic));
  ROS_ASSERT(private_nh.getParam("raw_right_encoder_count_topic", raw_right_encoder_count_topic));
  ROS_ASSERT(private_nh.getParam("encoder_timer_topic", encoder_timer_topic));
  ROS_ASSERT(private_nh.getParam("timer_period", timer_period));
  ROS_ASSERT(private_nh.getParam("encoder_count_publish_topic", encoder_count_publish_topic));

  encoder_raw_sub_left = nh.subscribe<std_msgs::Int64>(raw_left_encoder_count_topic, 1,
                                                       &ArduinoEncoderTimer::rawLeftEncoderCallback, this);
  encoder_raw_sub_right = nh.subscribe<std_msgs::Int64>(raw_right_encoder_count_topic, 1,
                                                        &ArduinoEncoderTimer::rawRightEncoderCallback, this);
  arduino_timer_pub = nh.advertise<std_msgs::Bool>(encoder_timer_topic, 1);
  encoder_count_pub = nh.advertise<agv::EncoderCount>(encoder_count_publish_topic, 1);

  encoder_timer = nh.createTimer(ros::Duration(timer_period), &ArduinoEncoderTimer::mainTimerCallback, this);
  previous_time = ros::Time::now();
  timer_msg.data = true;
  left_count_updated = right_count_updated = false;
}

void ArduinoEncoderTimer::mainTimerCallback(const ros::TimerEvent& timer_event)
{
  arduino_timer_pub.publish(timer_msg);
}

void ArduinoEncoderTimer::publish_to_encoder_topic()
{
  ros::Time current_time = ros::Time::now();
  ros::Duration timeDiff = current_time - previous_time;
  double timeDiffInSeconds = (timeDiff.nsec / 1000000000.0) + timeDiff.sec;
  previous_time = current_time;  // update time

  encoder_msg.left = (double)(left_count - prev_left_count);
  encoder_msg.right = (double)(right_count - prev_right_count);
  encoder_msg.dt = timeDiffInSeconds;
  encoder_msg.stamp = current_time;
  encoder_count_pub.publish(encoder_msg);

  prev_left_count = left_count;
  prev_right_count = right_count;
  right_count_updated = left_count_updated = false;
}

void ArduinoEncoderTimer::rawLeftEncoderCallback(const std_msgs::Int64::ConstPtr& raw_left_encoder_msg)
{
  left_count = raw_left_encoder_msg->data;
  left_count_updated = true;
  if (right_count_updated && left_count_updated)
  {
    publish_to_encoder_topic();
  }
}

void ArduinoEncoderTimer::rawRightEncoderCallback(const std_msgs::Int64::ConstPtr& raw_right_encoder_msg)
{
  right_count = raw_right_encoder_msg->data;
  right_count_updated = true;
  if (right_count_updated && left_count_updated)
  {
    publish_to_encoder_topic();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "arduino_encoder_timer");
  ArduinoEncoderTimer arduino_encoder_timer_obj;
  ros::spin();
  return 0;
}
