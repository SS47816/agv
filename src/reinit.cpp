// This node performs initialisation of buggy when it is started up. 

// 1) Performs automatic relocalisation.
//  1.1) Subscribe to /ndt_pose for the latest pose. Keep saving the latest pose to file, as the buggy moves. 
//  1.2) <Buggy is restarted/ restart ROS programs>
//  1.3) Reinitialisation of pose: Read from the file and Publish the last known location to set the pose

//TODO: There is an issue, that if you ctrl-c during the while loop, it will terminate only after some time.

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float64.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <iostream>
#include <fstream>

class Reinit
{
public:
  Reinit();

private:
  ros::NodeHandle nh;
  ros::Subscriber ndt_pose_sub;
  ros::Publisher initialpose_pub;


  void ndtPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& ndt_pose);
  void initialPoseInit(std::string file_name);

  //util function
  std::string getPath(std::string file_name);

  //ros params
  std::string file_name;
  std::string ndt_pose_topic;
  std::string initialpose_topic;
  std::string frame_id;

  std::string fullDir;
};

// constructor
Reinit::Reinit()
{
  ros::NodeHandle private_nh("~");

  ROS_ASSERT(private_nh.getParam("ndt_pose_topic", ndt_pose_topic));
  ROS_ASSERT(private_nh.getParam("initialpose_topic", initialpose_topic));
  ROS_ASSERT(private_nh.getParam("file_name", file_name));
  ROS_ASSERT(private_nh.getParam("frame_id", frame_id));

  //get the path to write to.
  fullDir = getPath(file_name);

  // Subscribe & Advertise
  initialpose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(initialpose_topic, 1);
  ndt_pose_sub = nh.subscribe(ndt_pose_topic, 1, &Reinit::ndtPoseCallback, this);

  //publish initial pose.
  initialPoseInit(file_name);
}

//Here we process messages from /ndt_pose. Just save to text file.
void Reinit::ndtPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& ndt_pose)
{
  //get data from msg
  double position_x = ndt_pose->pose.pose.position.x;
  double position_y = ndt_pose->pose.pose.position.y;
  double position_z = ndt_pose->pose.pose.position.z;
  double quaternion_x = ndt_pose->pose.pose.orientation.x;
  double quaternion_y = ndt_pose->pose.pose.orientation.y;
  double quaternion_z = ndt_pose->pose.pose.orientation.z;
  double quaternion_w = ndt_pose->pose.pose.orientation.w;
 

  //write to disk
  std::ofstream out_file;
  out_file.open(fullDir);
  if (out_file.is_open())
  {
    out_file
      << position_x << " "
      << position_y << " "
      << position_z << " "
      << quaternion_x << " "
      << quaternion_y << " "
      << quaternion_z << " "
      << quaternion_w << " "
      ;
    out_file.close();
    //ROS_INFO_STREAM("Reinit: wrote an ndt_pose to: " + fullDir);
  }
  else 
  {
    ROS_ERROR_STREAM("Reinit: Writing to file failed. Path: " + fullDir);
  }
  out_file.close();
}

//get the path of the text file
std::string Reinit::getPath(std::string file_name)
{
  const char* homeDir_c = getenv("HOME"); // "/home/agv"
  std::string homeDir = homeDir_c;
  std::string fullDir_ = homeDir + file_name;
  ROS_DEBUG_STREAM("file path: " + fullDir_);
  
  return fullDir_;
}

//Function that reads the file and publishes to /initial_pose. Should be called at start up.
void Reinit::initialPoseInit (std::string file_name) 
{
  //wait for subscriber (ndt node)
  while (initialpose_pub.getNumSubscribers() == 0) 
  {
    //TODO: maybe try to detect ctrl-c here and then exit the program
  } 

  geometry_msgs::PoseWithCovarianceStamped initialpose_msg;

  //init vars
  // double position_x;
  // double position_y;
  // double position_z;
  // double quaternion_x;
  // double quaternion_y;
  // double quaternion_z;
  // double quaternion_w;

  std::vector<std::string> list_of_inputs;
  //read the saved pose
  std::ifstream in_file;
  in_file.open(fullDir);
  if (in_file.is_open())
  {
    std::string read_data;
    while ( in_file >> read_data )
    {
      list_of_inputs.emplace_back(read_data);
    }
    ROS_INFO_STREAM("Reinit: read the ndt_pose in: " + fullDir);
    in_file.close();
  }
  else 
  {
    ROS_ERROR_STREAM("Reinit: reading from file failed. Does it exist? Path: " + fullDir);
    in_file.close();
    return;
  }

  //process the data read
  for (auto item : list_of_inputs)
  {
    ROS_DEBUG_STREAM(item);
  }
  initialpose_msg.pose.pose.position.x = stod(list_of_inputs.at(0));
  initialpose_msg.pose.pose.position.y = stod(list_of_inputs.at(1));
  initialpose_msg.pose.pose.position.z = stod(list_of_inputs.at(2)); 
  initialpose_msg.pose.pose.orientation.x = stod(list_of_inputs.at(3)); 
  initialpose_msg.pose.pose.orientation.y = stod(list_of_inputs.at(4));
  initialpose_msg.pose.pose.orientation.z = stod(list_of_inputs.at(5));
  initialpose_msg.pose.pose.orientation.w = stod(list_of_inputs.at(6));
  initialpose_msg.header.frame_id = frame_id;

  //publish to the initialpose topic
  initialpose_pub.publish(initialpose_msg);
  ROS_INFO_STREAM("Reinit: Published to : " + initialpose_topic);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "class_name_node");
  Reinit class_name_obj;
  ros::spin();
  return 0;
}