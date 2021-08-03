#include <math.h>
#include <vector>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <autoware_msgs/DetectedObject.h>
#include <autoware_msgs/DetectedObjectArray.h>

// #include <tf/tf.h>
// #include <tf/transform_listener.h>

class CameraProjector
{
public:
  CameraProjector();

  struct CameraParam
  {
    float half_width;       // [pixel]
    float half_height;      // [pixel]
    float half_fov_w;       // [rad]
    float half_fov_h;       // [rad]
    float center_x;         // [pixel]
    float center_y;         // [pixel]
    float depth_scale;      // [unit/m]
    float camera_height;    // distance from the ground [m]
    float min_depth;        // [m]
    float max_depth;        // [m]
  };

  struct Pixel
  {
    float depth;
    int x;
    int y;
  };

private:
  ros::NodeHandle nh;
  image_transport::Subscriber depth_sub;
  ros::Subscriber camera_objects_sub;
  ros::Subscriber lidar_objects_sub;
  ros::Publisher output_objects_pub;

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;
  std::string target_frame_;
  
  CameraParam camera_params_;
  autoware_msgs::DetectedObjectArray camera_objects_msg_;
  autoware_msgs::DetectedObjectArray lidar_objects_msg_;
  bool camera_objects_updated_;
  
  void depthImageCallback(const sensor_msgs::Image::ConstPtr& depth_msg);
  void cameraObjectsCallback(const autoware_msgs::DetectedObjectArray::ConstPtr& msg);
  void lidarObjectsCallback(const autoware_msgs::DetectedObjectArray::ConstPtr& msg);

  void projectObject(autoware_msgs::DetectedObject& object, const CameraParam& camera_params, const float* depths);
  float computeBBoxDepth(const float* depths, const CameraParam& camera_params, const Pixel& top_left, const Pixel& bot_right);
  bool retrieveDepth(const float* depths, const CameraParam& camera_params, CameraProjector::Pixel& pt);
  void transformObjectArray(autoware_msgs::DetectedObjectArray& msg, std::string target_frame, geometry_msgs::TransformStamped transformStamped);
};

CameraProjector::CameraProjector() : tf_listener(tf_buffer)
{
  ros::NodeHandle private_nh("~");
  image_transport::ImageTransport it(nh);
  int image_width, image_height;
  float image_fov_w, image_fov_h; // depth_scale, camera_height, min_depth, max_depth;
  std::string depth_image_topic;
  std::string camera_objects_topic;
  std::string lidar_objects_topic;
  std::string output_objects_topic;
  ROS_ASSERT(private_nh.getParam("depth_image_topic", depth_image_topic));
  ROS_ASSERT(private_nh.getParam("camera_objects_topic", camera_objects_topic));
  ROS_ASSERT(private_nh.getParam("lidar_objects_topic", lidar_objects_topic));
  ROS_ASSERT(private_nh.getParam("output_objects_topic", output_objects_topic));
  ROS_ASSERT(private_nh.getParam("target_frame", target_frame_));
  ROS_ASSERT(private_nh.getParam("image_width", image_width));
  ROS_ASSERT(private_nh.getParam("image_height", image_height));
  ROS_ASSERT(private_nh.getParam("image_fov_w", image_fov_w));
  ROS_ASSERT(private_nh.getParam("image_fov_h", image_fov_h));
  ROS_ASSERT(private_nh.getParam("depth_scale", camera_params_.depth_scale));
  ROS_ASSERT(private_nh.getParam("camera_height", camera_params_.camera_height));
  ROS_ASSERT(private_nh.getParam("min_depth", camera_params_.min_depth));
  ROS_ASSERT(private_nh.getParam("max_depth", camera_params_.max_depth));

  depth_sub = it.subscribe(depth_image_topic, 1, &CameraProjector::depthImageCallback, this);
  camera_objects_sub = nh.subscribe(camera_objects_topic, 1, &CameraProjector::cameraObjectsCallback, this);
  lidar_objects_sub = nh.subscribe(lidar_objects_topic, 1, &CameraProjector::lidarObjectsCallback, this);
  output_objects_pub = nh.advertise<autoware_msgs::DetectedObjectArray>(output_objects_topic, 1);

  // init camera info (using ZED @ HD720)
  camera_params_.half_width = image_width/2;
  camera_params_.half_height = image_height/2;
  camera_params_.half_fov_w = image_fov_w/2 * M_PI / 180.0;
  camera_params_.half_fov_h = image_fov_h/2 * M_PI / 180.0;
  camera_params_.center_x = (image_width - 1)/2;
  camera_params_.center_y = (image_height - 1)/2;

  camera_objects_updated_ = false;
};

void CameraProjector::depthImageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  if (camera_objects_updated_)
  {
    // save the depth image
    float* depths = (float*)(&msg->data[0]);
    // ROS_INFO("Camera Projector: depth image recieved! ");
    
    // project the 2D camera objects into 3D
    for (auto &object : camera_objects_msg_.objects)
    {
      projectObject(object, camera_params_, depths);
    }

    // transform the 3D camera objects into the target frame
    geometry_msgs::TransformStamped transformStamped;
    try
    {
      transformStamped = tf_buffer.lookupTransform(target_frame_, camera_objects_msg_.header.frame_id, ros::Time(0));
      transformObjectArray(camera_objects_msg_, target_frame_, transformStamped);
    }
    catch (tf2::TransformException &ex) 
    {
      ROS_WARN("%s", ex.what());
    }

    camera_objects_updated_ = false;
  }

  autoware_msgs::DetectedObjectArray output_objects_msg_ = camera_objects_msg_;

  // concatenate the camera objects and the lidar objects
  if (!lidar_objects_msg_.objects.empty())
  {
    output_objects_msg_.objects.insert(
      output_objects_msg_.objects.end(),
      lidar_objects_msg_.objects.begin(),
      lidar_objects_msg_.objects.end()
    );
  }

  output_objects_pub.publish(output_objects_msg_);
};

void CameraProjector::cameraObjectsCallback(const autoware_msgs::DetectedObjectArray::ConstPtr& msg)
{
  if (msg->objects.size() <= 0)
  {
    ROS_WARN("[Camera Projector] Zero Camera 2D Objects recieved! ");
    camera_objects_updated_ = false;
  }
  else
  {
    camera_objects_msg_ = std::move(*msg);
    // ROS_INFO("[Camera Projector] Detected 2D Objects recieved! ");
    camera_objects_updated_ = true;
  }
};

void CameraProjector::lidarObjectsCallback(const autoware_msgs::DetectedObjectArray::ConstPtr& msg)
{
  if (msg->objects.size() <= 0)
  {
    ROS_WARN("[Camera Projector] Zero LiDAR 3D Objects recieved! ");
  }
  else
  {
    lidar_objects_msg_ = std::move(*msg);
    // ROS_INFO("[Camera Projector] LiDAR 3D Objects recieved! ");
  }
};

void CameraProjector::projectObject(autoware_msgs::DetectedObject& object, 
                                    const CameraProjector::CameraParam& camera_params, 
                                    const float* depths)
{
  // object's key pixels in image frame (x -> width, y -> height) in [pixels]
  CameraProjector::Pixel top_left, bot_right;
  top_left.x = object.x;
  top_left.y = object.y;
  bot_right.x = object.x + object.width;
  bot_right.y = object.y + object.height;

  // retrieve the depth of the object
  const float object_d = computeBBoxDepth(depths, camera_params, top_left, bot_right);
  std::string object_name = object.label + " " + std::to_string(object.id);
  if (object_d >= camera_params.min_depth && object_d <= camera_params.max_depth)
  {
    ROS_INFO("[Camera Projector] %s depth = %g m", object_name.c_str(), object_d);
  }
  else
  {
    ROS_WARN("[Camera Projector] %s depth value out of range", object_name.c_str());
  }

  // object's 2d bbox in camera (real-world) frame (x -> forward, y -> leftward, z -> upward)
  // top left corner
  const int top_left_y_c = camera_params.half_width - top_left.x;
  const int top_left_z_c = camera_params.half_height - top_left.y;
  // bot right corner
  const int bot_right_y_c = camera_params.half_width - bot_right.x;
  const int bot_right_z_c = camera_params.half_height - bot_right.y;

  // object's 3d bbox dimension in camera frame
  const float top_left_y = tan(top_left_y_c / camera_params.half_width * camera_params_.half_fov_w) * object_d;
  const float top_left_z = tan(top_left_z_c / camera_params.half_height * camera_params_.half_fov_h) * object_d;
  const float bot_right_y = tan(bot_right_y_c / camera_params.half_width * camera_params_.half_fov_w) * object_d;
  const float bot_right_z = tan(bot_right_z_c / camera_params.half_height * camera_params_.half_fov_h) * object_d;

  const float object_w = top_left_y - bot_right_y;
  const float object_h = top_left_z + camera_params.camera_height;

  // add in the 3d bbox properties to the object
  object.dimensions.x = object_w;
  object.dimensions.y = object_w;
  object.dimensions.z = object_h;
  tf2::Quaternion myQuaternion;
  myQuaternion.setRPY(0, 0, 0);
  myQuaternion.normalize();
  object.pose.orientation = tf2::toMsg(myQuaternion);
  object.pose.position.x = object_d + object_w/2;
  object.pose.position.y = (top_left_y + bot_right_y)/2;
  // if most of the object's height is in view
  if (0.85*camera_params.camera_height + bot_right_z <= 0)
  {
    // use the 2d bbox center as its 3d bbox center
    object.pose.position.z = (top_left_z + bot_right_z)/2;
  }
  else
  {
    // enlongate the object's 3d bbox to reach the ground
    object.pose.position.z = (top_left_z - camera_params.camera_height)/2;
  }
  
  object.pose_reliable = true;
  object.velocity_reliable = false;
  object.acceleration_reliable = false;
  object.valid = true;
  // object.color = ;

  // add in the convex hull properties to the object
  object.convex_hull.header = object.header;
  geometry_msgs::Point32 tl_point, tr_point, br_point, bl_point;
  tl_point.x = object_d + object_w;
  tl_point.y = top_left_y;
  tl_point.z = object.pose.position.z - object.dimensions.z/2;
  tr_point.x = tl_point.x;
  tr_point.y = bot_right_y;
  tr_point.z = tl_point.z;
  br_point.x = object_d;
  br_point.y = bot_right_y;
  br_point.z = tl_point.z;
  bl_point.x = object_d;
  bl_point.y = top_left_y;
  bl_point.z = tl_point.z;

  object.convex_hull.polygon.points.clear();
  object.convex_hull.polygon.points.emplace_back(tl_point);
  object.convex_hull.polygon.points.emplace_back(tr_point);
  object.convex_hull.polygon.points.emplace_back(br_point);
  object.convex_hull.polygon.points.emplace_back(bl_point);
};

float CameraProjector::computeBBoxDepth(const float* depths, const CameraProjector::CameraParam& camera_params, 
                                        const CameraProjector::Pixel& top_left, const CameraProjector::Pixel& bot_right)
{
  // some selected key points to represent the 2d bbox's depth
  CameraProjector::Pixel center, top, left, bot, right; 
  center.x = (top_left.x + bot_right.x)/2;
  center.y = (top_left.y + bot_right.y)/2;
  top.x = center.x;
  top.y = (center.y + top_left.y)/2;
  bot.x = center.x;
  bot.y = (center.y + bot_right.y)/2;
  left.x = (center.x + top_left.x)/2;
  left.y = center.y;
  right.x = (center.x + bot_right.x)/2;
  right.y = center.y;

  std::vector<CameraProjector::Pixel> keypoints {center, top, bot, left, right};
  float sum = 0.0;
  size_t count = 0;
  for (auto& keypoint : keypoints)
  {
    if(retrieveDepth(depths, camera_params, keypoint))
    {
      sum += keypoint.depth;
      count++;
    }
  }

  if (count <= 0) return -1.0;
  return sum/count;
}

bool CameraProjector::retrieveDepth(const float* depths, const CameraProjector::CameraParam& camera_params, 
                                    CameraProjector::Pixel& pt)
{
  // Linear index of the center pixel
  const int idx = pt.x + static_cast<int>(std::round(2*camera_params.half_width)) * pt.y;
  // retrieve the depth at the object's center
  const auto depth = depths[idx];
  if (std::isnan(depth) || std::isinf(depth))
  {
    return false;
  }
  else
  {
    pt.depth = depth * camera_params.depth_scale;
  }

  return true;
};

void CameraProjector::transformObjectArray(autoware_msgs::DetectedObjectArray& msg, std::string target_frame, geometry_msgs::TransformStamped transformStamped)
{
  msg.header.frame_id = target_frame;
  for (auto &object : msg.objects)
  {
    object.header.frame_id = msg.header.frame_id;
    // transform object's pose
    tf2::doTransform(object.pose, object.pose, transformStamped);
    // transform object's convex hull
    object.convex_hull.header.frame_id = object.header.frame_id;
    for (auto &point32 : object.convex_hull.polygon.points)
    {
      geometry_msgs::Point point;
      point.x = point32.x;
      point.y = point32.y;
      point.z = point32.z;
      tf2::doTransform(point, point, transformStamped);
      point32.x = point.x;
      point32.y = point.y;
      point32.z = point.z;
    }
  }
}


/**
 * Node main function
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "camera_projector_node");
  CameraProjector camera_projector;
  ros::spin();
  return EXIT_SUCCESS;
}
