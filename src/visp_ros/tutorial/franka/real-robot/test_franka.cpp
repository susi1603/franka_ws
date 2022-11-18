// #include <iostream>
// #include <visp3/core/vpConfig.h>
// #if defined(VISP_HAVE_FRANKA)
// #include <visp3/robot/vpRobotFranka.h>
// int main(int argc, char **argv)
// {
//   std::string robot_ip = "172.16.0.2";
//   try {
//     vpRobotFranka robot;
//     robot.connect(robot_ip);
//     std::cout << "WARNING: This example will move the robot! "
//               << "Please make sure to have the user stop button at hand!" << std::endl
//               << "Press Enter to continue..." << std::endl;
//     std::cin.ignore();

//     vpColVector initial_q = {-0.2113152342, 0.211263765,  0.5191193356,  -1.486009226, -0.08664920681, 1.736819775, -0.4036056139};

//     /*
//     * Get the actual joint position
//     */

//     vpColVector ee_q(7, 0);
//     robot.setRobotState(vpRobot::STATE_POSITION_CONTROL);
//     robot.getPosition( vpRobot::JOINT_STATE, ee_q);
//     std::cout << "EE joint position: " << ee_q.t()<< std::endl;


//     /*
//      * Move to current position
//      */

//     // vpColVector q(7, 0);
//     // q = ee_q;
//     robot.setRobotState(vpRobot::STATE_POSITION_CONTROL);
//     std::cout << "Move to joint position: " << initial_q.t() << std::endl;
//     robot.setPosition(vpRobot::JOINT_STATE, initial_q);

//     /*
//      * Move to a safe position
//      */

    

//     // vpColVector q(7, 0);
//     // q[3] = -M_PI_2;
//     // q[5] = M_PI_2;
//     // q[6] = M_PI_4;
//     // robot.setRobotState(vpRobot::STATE_POSITION_CONTROL);
//     // std::cout << "Move to joint position: " << q.t() << std::endl;
//     // robot.setPosition(vpRobot::JOINT_STATE, q);


//   }
//   catch(const vpException &e) {
//     std::cout << "ViSP exception: " << e.what() << std::endl;
//     return EXIT_FAILURE;
//   }
//   catch(const franka::NetworkException &e) {
//     std::cout << "Franka network exception: " << e.what() << std::endl;
//     std::cout << "Check if you are connected to the Franka robot"
//               << " or if you specified the right IP using --ip command"
//               << " line option set by default to 192.168.1.1. " << std::endl;
//     return EXIT_FAILURE;
//   }
//   catch(const std::exception &e) {
//     std::cout << "Franka exception: " << e.what() << std::endl;
//     return EXIT_FAILURE;
//   }
//   std::cout << "The end" << std::endl;
//   return EXIT_SUCCESS;
// }
// #else
// int main()
// {
//   std::cout << "ViSP is not build with libfranka..." << std::endl;
// }
// #endif

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string> 

#include <ros/ros.h>
#include <tf/transform_listener.h>

#define CONFIG_FILE_PATH_NEW "./src/visp_ros/tutorial/franka/real-robot/config/configFile.config"
#define CONFIG_FILE_PATH_OLD "./src/visp_ros/tutorial/franka/real-robot/config/configFile~.config"

tf::StampedTransform eedMo;
tf::StampedTransform wMee;
using namespace std;

void setToConfigFile(){

    std::rename(CONFIG_FILE_PATH_NEW, CONFIG_FILE_PATH_OLD);
    std::ifstream in(CONFIG_FILE_PATH_OLD);
    std::ofstream out(CONFIG_FILE_PATH_NEW, std::ofstream::trunc);

    if(!in || !out)
    {
        cout << "Error opening files!" << endl;
    }
    std::string param;
    double value;

    while (!in.eof())
    {
      in >> param;
      in >> value;
      if (param == "EEDMO_TRANSLATION_X")
      {
        out << param << " " << eedMo.getOrigin().x() << endl;
      }
      if (param == "EEDMO_TRANSLATION_Y")
      {
        out << param << " " << eedMo.getOrigin().y() << endl;
      }
      if (param == "EEDMO_TRANSLATION_Z")
      {
        out << param << " " << eedMo.getOrigin().z() << endl;
      }
      if (param == "EEDMO_QUARTENION_X")
      {
        out << param  << " " << eedMo.getRotation().getX() << endl;
      }
      if (param == "EEDMO_QUARTENION_Y")
      {
        out << param  << " " <<  eedMo.getRotation().getY() << endl;
      }
      if (param == "EEDMO_QUARTENION_Z")
      {
       out << param  << " " <<  eedMo.getRotation().getZ() << endl;
      }
      if (param == "EEDMO_QUARTENION_W")
      {
        out << param  << " " <<  eedMo.getRotation().getW() << endl;
      }
      if (param == "WMLZERO_TRANSLATION_X")
      {
        out << param  << " " <<  value << endl;
      }
      if (param == "WMLZERO_TRANSLATION_Y")
      {
       out << param  << " " <<  value << endl;
      }
      if (param == "WMLZERO_TRANSLATION_Z")
      {
        out << param  << " " <<  value << endl;
      }
      if (param == "WMLZERO_QUARTENION_X")
      {
        out << param  << " " <<  value << endl;
      }
      if (param == "WMLZERO_QUARTENION_Y")
      {
        out << param  << " " <<  value << endl;
      }
      if (param == "WMLZERO_QUARTENION_Z")
      {
        out << param  << " " <<  value << endl;
      }
      if (param == "WMLZERO_QUARTENION_W")
      {
        out << param  << " " <<  value << endl;
      }
      if (param == "STARTING_WMEE_X")
      {
        out << param  << " " <<  wMee.getOrigin().x() << endl;
      }
      if (param == "STARTING_WMEE_Y")
      {
        out << param  << " " << wMee.getOrigin().y() << endl;
      }
      if (param == "STARTING_WMEE_Z")
      {
        out << param  << " " << wMee.getOrigin().z() << endl;
      }
        if (param == "BOUNDING_BOX_X_forward")
      {
        out << param  << " " <<  value << endl;
      }
      if (param == "BOUNDING_BOX_X_backward")
      {
        out << param  << " " <<  value << endl;
      }
      if (param == "BOUNDING_BOX_Y_left")
      {
        out << param  << " " <<  value << endl;
      }
      if (param == "BOUNDING_BOX_Y_right")
      {
        out << param  << " " <<  value << endl;
      }
      if (param == "BOUNDING_BOX_Z_up")
      {
       out << param  << " " <<  value << endl;
      }
      if (param == "BOUNDING_BOX_Z_down")
      {
        out << param  << " " <<  value ;
      }
    }
    in.close();
    out.close();
  }



int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;

  tf::TransformListener listener;

  ros::Rate rate(10.0);
  while (node.ok()){
    try{
      listener.lookupTransform("/panda_EE", "/object",  
                               ros::Time(0), eedMo);
      listener.lookupTransform("/world", "/panda_EE",  
                               ros::Time(0), wMee);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    rate.sleep();
  }
  setToConfigFile();

  return 0;
};



