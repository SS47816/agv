/*
 * This program outputs the odometry information based on
 * rate of change of wheel encoders and dt
 *
 * Standard units of measurement are:
 * Distance => Metres
 * Time => Seconds
 * Angle => Radians
 *
 * Includes: IMU
 */

#include <ros/ros.h>
#include <ros/console.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <agv/EncoderCount.h>
#include <cmath>

#define RADIAN_TO_DEG_CONST 57.295779513
#define DEG_TO_RADIAN_CONST 0.01745329251

class EncoderOdometrySystem
{
public:
  EncoderOdometrySystem();

private:
  ros::NodeHandle nh;

  ros::Subscriber encoder_count_sub;
  ros::Subscriber imu_sub;
  ros::Publisher odom_pub;
  tf::TransformBroadcaster odom_broadcaster;

  nav_msgs::Odometry odom_msg;
  
  double left_count;
  double right_count;
  double odom_x;
  double odom_y;
  bool use_imu_yaw;
  double curr_imu_yaw;
  double prev_imu_yaw;

  // vehicle specific params
  double AXLE_DIST;       // front wheel to rear wheel distance [m]
  double WHEEL_BASE;      // left wheel to right wheel distance [m]
  double L_WHEEL_DIA;     // left wheel diameter [m]
  double R_WHEEL_DIA;     // right wheel diameter [m]
  double GEAR_RATIO;      // encoder gear : pinion ratio
  int P_R;                // encoder pulses per round
  double L_COUNT_TO_DIST; // const to be calculated [m]
  double R_COUNT_TO_DIST; // const to be calculated [m]

  void getCountToDistConst();
  void encoderCallback(const agv::EncoderCount::ConstPtr& encoder_msg);
  void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg);
};

EncoderOdometrySystem::EncoderOdometrySystem()
{
  ros::NodeHandle private_nh("~");

  std::string encoder_count_topic;
  std::string imu_topic;
  std::string encoder_odom_topic;

  ROS_ASSERT(private_nh.getParam("encoder_count_topic", encoder_count_topic));
  ROS_ASSERT(private_nh.getParam("imu_topic", imu_topic));
  ROS_ASSERT(private_nh.getParam("encoder_odom_topic", encoder_odom_topic));
  ROS_ASSERT(private_nh.getParam("odom_x", odom_x));
  ROS_ASSERT(private_nh.getParam("odom_y", odom_y));
  ROS_ASSERT(private_nh.getParam("use_imu_yaw", use_imu_yaw));
  ROS_ASSERT(private_nh.getParam("axle_distance", AXLE_DIST));
  ROS_ASSERT(private_nh.getParam("wheel_base", WHEEL_BASE));
  ROS_ASSERT(private_nh.getParam("left_wheel_diameter", L_WHEEL_DIA));
  ROS_ASSERT(private_nh.getParam("right_wheel_diameter", R_WHEEL_DIA));
  ROS_ASSERT(private_nh.getParam("gear_ratio", GEAR_RATIO));
  ROS_ASSERT(private_nh.getParam("pulses_per_round", P_R));

  getCountToDistConst();

  encoder_count_sub = nh.subscribe<agv::EncoderCount>(encoder_count_topic, 1, &EncoderOdometrySystem::encoderCallback, this);
  imu_sub = nh.subscribe(imu_topic, 1, &EncoderOdometrySystem::imuCallback, this);
  odom_pub = nh.advertise<nav_msgs::Odometry>(encoder_odom_topic, 1);

  left_count = 0;
  right_count = 0;

  odom_msg.header.frame_id = "odom";
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0);
  odom_msg.pose.pose.orientation = odom_quat;
  odom_msg.pose.pose.position.x = odom_x;
  odom_msg.pose.pose.position.y = odom_y;
  odom_msg.twist.twist.linear.x = 0;
  odom_msg.twist.twist.linear.y = 0;
  odom_msg.twist.twist.angular.z = 0;

  odom_pub.publish(odom_msg);
}

void EncoderOdometrySystem::getCountToDistConst()
{
  /*
   * COUNT_TO_DIST = gear ratio * PI * wheel diameter / encoder count per rev / encoder library quarter cycle /
   * bias = (12 / 74) * PI * 0.508 / 1000 / 4 / 1.04 = 0.00006231137
   */
  L_COUNT_TO_DIST = L_WHEEL_DIA * M_PI / GEAR_RATIO / P_R;
  R_COUNT_TO_DIST = L_WHEEL_DIA * M_PI / GEAR_RATIO / P_R;
}

void EncoderOdometrySystem::imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
  prev_imu_yaw = curr_imu_yaw;
  tf::Quaternion q_imu(imu_msg->orientation.x, imu_msg->orientation.y, imu_msg->orientation.z, imu_msg->orientation.w);
  tf::Matrix3x3 m_imu(q_imu);
  double roll, pitch;
  m_imu.getRPY(roll, pitch, curr_imu_yaw);
}

void EncoderOdometrySystem::encoderCallback(const agv::EncoderCount::ConstPtr& msg)
{
  /*
   * Updates the difference in counts in dt time
   * Unit of counts is number of revolutions of encoders
   * Unit of dt is seconds, typ 0.1s
   */
  left_count = msg->left;
  right_count = msg->right;
  const double dt = msg->dt;
  
  const double dist_left = left_count * L_COUNT_TO_DIST;
  const double dist_right = right_count * R_COUNT_TO_DIST;
  const double dist_average = (dist_left + dist_right) / 2.0;
  
  double d_theta, mid_theta;
  if (use_imu_yaw)
  {
    d_theta = curr_imu_yaw - prev_imu_yaw;
    mid_theta = (curr_imu_yaw + prev_imu_yaw) / 2.0;
  }
  
  const double v = dist_average / dt;
  const double d_x = cos(d_theta) * dist_average;
  const double d_y = sin(d_theta) * dist_average;
  odom_x += d_x * cos(prev_imu_yaw) - d_y * sin(prev_imu_yaw);
  odom_y += d_x * sin(prev_imu_yaw) + d_y * cos(prev_imu_yaw);

  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(curr_imu_yaw);

  // publish the transform (if not using ekf's transform)
  // geometry_msgs::TransformStamped odom_trans;
  // odom_trans.header.stamp = ros::Time::now();
  // odom_trans.header.frame_id = "odom";
  // odom_trans.child_frame_id = "base_link";
  // odom_trans.transform.translation.x = odom_x;
  // odom_trans.transform.translation.y = odom_y;
  // odom_trans.transform.translation.z = 0.0;
  // odom_trans.transform.rotation = odom_quat;
  // odom_broadcaster.sendTransform(odom_trans);

  // publish the message
  odom_msg.header.stamp = ros::Time::now();
  odom_msg.header.frame_id = "odom";
  odom_msg.pose.pose.orientation = odom_quat;
  odom_msg.pose.pose.position.x = odom_x;
  odom_msg.pose.pose.position.y = odom_y;
  odom_msg.twist.twist.linear.x = v;
  odom_msg.twist.twist.linear.y = 0;
  odom_msg.twist.twist.angular.z = d_theta / dt;
  
  odom_pub.publish(odom_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "encoder_odometry_system_node");
  EncoderOdometrySystem encoder_odometry_system_obj;
  ros::spin();
  return 0;
}
