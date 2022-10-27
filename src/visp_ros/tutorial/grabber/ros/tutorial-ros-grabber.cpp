#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <art_publisher/marker.h>
#include <art_publisher/body.h>
#include <tf/transform_broadcaster.h>

double x_pos;
double y_pos;
double z_pos;
double x_or;
double y_or;
double z_or;
double w_or;

void artCallback (const art_publisher::body::ConstPtr &msg)
{
    
    x_pos = msg->bodies[2].pose.position.x/1000;
    y_pos = msg->bodies[2].pose.position.y/1000;
    z_pos = msg->bodies[2].pose.position.z/1000;
    x_or  = msg->bodies[2].pose.orientation.x; 
    y_or  = msg->bodies[2].pose.orientation.y; 
    z_or  = msg->bodies[2].pose.orientation.z; 
    w_or  = msg->bodies[2].pose.orientation.w; 
    ROS_INFO("%f", x_pos);
    ROS_INFO("%f", y_pos);
    ROS_INFO("%f", z_pos);
    ROS_INFO("%f", x_or );
    ROS_INFO("%f", y_or );
    ROS_INFO("%f", z_or );
    ROS_INFO("%f", w_or );

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(x_pos, y_pos,z_pos ) );
    tf::Quaternion q(x_or,y_or,z_or,w_or);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "calib_tool_ee"));
}

int main( int argc, char** argv )
{
  
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
//   ros::Rate r(1);
  ros::Subscriber sub = n.subscribe("ARTBody", 1000, artCallback);
//   ros::spin();


  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  uint32_t shape = visualization_msgs::Marker::CUBE;

  ros::Rate loop_rate(10);
  float f = 0.0;

  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/object_frame";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x =    x_pos;
    marker.pose.position.y =    y_pos;
    marker.pose.position.z =    z_pos;
    marker.pose.orientation.x = x_or;
    marker.pose.orientation.y = y_or;
    marker.pose.orientation.z = z_or;
    marker.pose.orientation.w = w_or;


    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();
    // Create the vertices for the points and lines
    // for (uint32_t i = 0; i < 4; ++i)
    // {
    // float y = 5 * sin(f + i / 100.0f * 2 * M_PI);
    // float z = 5 * cos(f + i / 100.0f * 2 * M_PI);

    // geometry_msgs::Point p;
    // p.x = (int32_t)i - 50;
    // p.y = y;
    // p.z = z;

    // marker.points.push_back(p);
    // p.z += 1.0;
    // }
    // while (marker_pub.getNumSubscribers() < 1)
    // {
    //   if (!ros::ok())
    //   {
    //     return 0;
    //   }
    //   ROS_WARN_ONCE("Please create a subscriber to the marker");
    //   sleep(1);
    // }
    marker_pub.publish(marker);
    ros::spinOnce();
    loop_rate.sleep();
    // r.sleep();
    // f += 0.04;
  }
}
