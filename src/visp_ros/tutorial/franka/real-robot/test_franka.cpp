#include <iostream>
#include <visp3/core/vpConfig.h>
#if defined( VISP_HAVE_FRANKA )
#include <visp3/robot/vpRobotFranka.h>
int
main( int argc, char **argv )
{
  std::string robot_ip = "172.16.0.2";
  try
  {
    vpRobotFranka robot;
    robot.connect( robot_ip );
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    vpColVector initial_q = { -0.2113152342,  0.211263765, 0.5191193356, -1.486009226,
                              -0.08664920681, 1.736819775, -0.4036056139 };

    /*
     * Get the actual joint position
     */

    vpColVector ee_q( 7, 0 );
    robot.setRobotState( vpRobot::STATE_POSITION_CONTROL );
    robot.getPosition( vpRobot::JOINT_STATE, ee_q );
    std::cout << "EE joint position: " << ee_q.t() << std::endl;

    /*
     * Move to current position
     */

    // vpColVector q(7, 0);
    // q = ee_q;
    // robot.setRobotState(vpRobot::STATE_POSITION_CONTROL);
    // std::cout << "Move to joint position: " << initial_q.t() << std::endl;
    // robot.setPosition(vpRobot::JOINT_STATE, initial_q);

    /*
     * Move to a safe position
     */

    // vpColVector q(7, 0);
    // q[3] = -M_PI_2;
    // q[5] = M_PI_2;
    // q[6] = M_PI_4;
    // robot.setRobotState(vpRobot::STATE_POSITION_CONTROL);
    // std::cout << "Move to joint position: " << q.t() << std::endl;
    // robot.setPosition(vpRobot::JOINT_STATE, q);
  }
  catch ( const vpException &e )
  {
    std::cout << "ViSP exception: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }
  catch ( const franka::NetworkException &e )
  {
    std::cout << "Franka network exception: " << e.what() << std::endl;
    std::cout << "Check if you are connected to the Franka robot"
              << " or if you specified the right IP using --ip command"
              << " line option set by default to 192.168.1.1. " << std::endl;
    return EXIT_FAILURE;
  }
  catch ( const std::exception &e )
  {
    std::cout << "Franka exception: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << "The end" << std::endl;
  return EXIT_SUCCESS;
}
#else
int
main()
{
  std::cout << "ViSP is not build with libfranka..." << std::endl;
}
#endif
