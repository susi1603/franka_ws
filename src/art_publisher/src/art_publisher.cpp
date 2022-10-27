#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "DTrackSDK.hpp"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include "art_publisher/marker.h"
#include <art_publisher/body.h>

static DTrackSDK* dt = NULL;
unsigned short PORT = 21000; // check the ART GUI and set this value


/**
 * Refer to DTrackSDK/examples/example_listening.cpp
 */
static bool data_error_to_console()
{
    if ( dt->getLastDataError() == DTrackSDK::ERR_TIMEOUT )
    {
        std::cout << "--- timeout while waiting for tracking data" << std::endl;
        return false;
    }

    if ( dt->getLastDataError() == DTrackSDK::ERR_NET )
    {
        std::cout << "--- error while receiving tracking data" << std::endl;
        return false;
    }

    if ( dt->getLastDataError() == DTrackSDK::ERR_PARSE )
    {
        std::cout << "--- error while parsing tracking data" << std::endl;
        return false;
    }

    return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "body_publisher");
  ros::NodeHandle n;
  ros::Publisher body_pub = n.advertise<art_publisher::body>("ARTBody", 1000);
  ros::Rate loop_rate(100);

  art_publisher::body body_sequence;
  geometry_msgs::PoseStamped pose_single;

  ROS_INFO("Start Publishing AR Tracker data...");

  dt = new DTrackSDK(PORT);
  ROS_INFO_STREAM("Listening at: " << dt->getDataPort());
  if (!dt->isDataInterfaceValid()){
      ROS_ERROR("DTrackSDK init error");
  }

  dt->setDataTimeoutUS(3000000);
  dt->setDataBufferSize(10000);
  ROS_INFO_STREAM("Frame Info : " << dt->getFrameCounter());
  ROS_INFO_STREAM("Time Stamp Info : " << dt->getTimeStamp());

  while (ros::ok())
  {
      if(dt->receive()){
          ROS_INFO("---------------------------------");
      }
      else{
          ROS_ERROR("No Data received");
      }
      ROS_INFO_STREAM("Number of body is: # " << dt->getNumBody());

      for(int i = 0; i < dt->getNumBody(); i++){
          const DTrackBody* body = dt->getBody(i);
          if(body==NULL){
              std::cout << "ERROR: invalid body index" << i << std::endl;
              break;
          }

          ROS_INFO_STREAM("Body ID: " << body->id);
          pose_single.header.seq = body-> id;
          pose_single.pose.position.x = body->loc[0];
          pose_single.pose.position.y = body->loc[1];
          pose_single.pose.position.z = body->loc[2];
          DTrackQuaternion quat = body->getQuaternion(); 
          pose_single.pose.orientation.w = quat.w;
          pose_single.pose.orientation.x = quat.x;
          pose_single.pose.orientation.y = quat.y;
          pose_single.pose.orientation.z = quat.z;

          ROS_INFO_STREAM("Body position data: x: "
                           << pose_single.pose.position.x << ", y: "
                           << pose_single.pose.position.y << ", z: "
                           << pose_single.pose.position.z);

          ROS_INFO_STREAM("Body orientation data: w:"
                           << pose_single.pose.orientation.w << ", x: "
                           << pose_single.pose.orientation.x << ", y: "
                           << pose_single.pose.orientation.y << ", z: "
                           << pose_single.pose.orientation.z);

          body_sequence.bodies.push_back(pose_single);
      }

      /* ROS_INFO_STREAM("# of bodies: " << body_sequence.bodies.size()); */
      body_pub.publish(body_sequence);
      body_sequence.bodies.clear();

      ros::spinOnce();
      loop_rate.sleep();
  }

  delete dt;
  return 0;
}
