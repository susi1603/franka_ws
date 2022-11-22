#include <fstream>
#include <iostream>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>

#include <ros/ros.h>
#include <tf/transform_listener.h>

#define CONFIG_FILE_PATH_NEW "./src/visp_ros/tutorial/franka/real-robot/config/configFile.config"
#define CONFIG_FILE_PATH_OLD "./src/visp_ros/tutorial/franka/real-robot/config/configFile~.config"

/*

For calibration steps:
1. cd ~/franka_ros_ws
2. roscore
4. rosrun art_publisher art_publisher
3. roslaunch franka_control franka_control_isolated.launch
5. cd ~/ros/franka_ws
6. rosrun visp_ros calibrate

*/

tf::StampedTransform eedMo;
tf::StampedTransform wMee;
tf::StampedTransform wMo;
using namespace std;

void
setToConfigFile()
{

  std::rename( CONFIG_FILE_PATH_NEW, CONFIG_FILE_PATH_OLD );
  std::ifstream in( CONFIG_FILE_PATH_OLD );
  std::ofstream out( CONFIG_FILE_PATH_NEW, std::ofstream::trunc );

  if ( !in || !out )
  {
    cout << "Error opening files!" << endl;
  }
  std::string param;
  double value;

  while ( !in.eof() )
  {
    in >> param;
    in >> value;
    if ( param == "EEDMO_TRANSLATION_X" )
    {
      out << param << " " << eedMo.getOrigin().x() << endl;
    }
    if ( param == "EEDMO_TRANSLATION_Y" )
    {
      out << param << " " << eedMo.getOrigin().y() << endl;
    }
    if ( param == "EEDMO_TRANSLATION_Z" )
    {
      out << param << " " << eedMo.getOrigin().z() << endl;
    }
    if ( param == "EEDMO_QUARTENION_X" )
    {
      out << param << " " << eedMo.getRotation().getX() << endl;
    }
    if ( param == "EEDMO_QUARTENION_Y" )
    {
      out << param << " " << eedMo.getRotation().getY() << endl;
    }
    if ( param == "EEDMO_QUARTENION_Z" )
    {
      out << param << " " << eedMo.getRotation().getZ() << endl;
    }
    if ( param == "EEDMO_QUARTENION_W" )
    {
      out << param << " " << eedMo.getRotation().getW() << endl;
    }
    if ( param == "WMLZERO_TRANSLATION_X" )
    {
      out << param << " " << value << endl;
    }
    if ( param == "WMLZERO_TRANSLATION_Y" )
    {
      out << param << " " << value << endl;
    }
    if ( param == "WMLZERO_TRANSLATION_Z" )
    {
      out << param << " " << value << endl;
    }
    if ( param == "WMLZERO_QUARTENION_X" )
    {
      out << param << " " << value << endl;
    }
    if ( param == "WMLZERO_QUARTENION_Y" )
    {
      out << param << " " << value << endl;
    }
    if ( param == "WMLZERO_QUARTENION_Z" )
    {
      out << param << " " << value << endl;
    }
    if ( param == "WMLZERO_QUARTENION_W" )
    {
      out << param << " " << value << endl;
    }
    if ( param == "STARTING_WMEE_X" )
    {
      out << param << " " << wMee.getOrigin().x() << endl;
    }
    if ( param == "STARTING_WMEE_Y" )
    {
      out << param << " " << wMee.getOrigin().y() << endl;
    }
    if ( param == "STARTING_WMEE_Z" )
    {
      out << param << " " << wMee.getOrigin().z() << endl;
    }
    if ( param == "BOUNDING_BOX_X_forward" )
    {
      out << param << " " << value << endl;
    }
    if ( param == "BOUNDING_BOX_X_backward" )
    {
      out << param << " " << value << endl;
    }
    if ( param == "BOUNDING_BOX_Y_left" )
    {
      out << param << " " << value << endl;
    }
    if ( param == "BOUNDING_BOX_Y_right" )
    {
      out << param << " " << value << endl;
    }
    if ( param == "BOUNDING_BOX_Z_up" )
    {
      out << param << " " << value << endl;
    }
    if ( param == "BOUNDING_BOX_Z_down" )
    {
      out << param << " " << value << endl;
    }

    tf::Quaternion q_tf(wMo.getRotation().getX(), wMo.getRotation().getY(), wMo.getRotation().getZ(), wMo.getRotation().getW());
    tfScalar yaw_tf, pitch_tf, roll_tf;
    tf::Matrix3x3 mat(q_tf);
    mat.getEulerYPR(yaw_tf,pitch_tf,roll_tf);

    if ( param == "WMO_R" )
    {
      out << param << " " << roll_tf << endl;
    }
    if ( param == "WMO_P" )
    {
      out << param << " " << pitch_tf << endl;
    }
    if ( param == "WMO_Y" )
    {
      out << param << " " << yaw_tf;
    }

  }
  in.close();
  out.close();
}

int
main( int argc, char **argv )
{
  ros::init( argc, argv, "my_tf_listener" );

  ros::NodeHandle node;

  tf::TransformListener listener;

  ros::Rate rate( 10.0 );
  while ( node.ok() )
  {
    try
    {
      listener.lookupTransform( "/panda_EE", "/object", ros::Time( 0 ), eedMo );
      listener.lookupTransform( "/world", "/panda_EE", ros::Time( 0 ), wMee );
      listener.lookupTransform( "/world", "/object", ros::Time( 0 ), wMo );
    }
    catch ( tf::TransformException ex )
    {
      ROS_ERROR( "%s", ex.what() );
      ros::Duration( 1.0 ).sleep();
    }
    rate.sleep();
  }
  setToConfigFile();
  return 0;
};
