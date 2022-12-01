// #include <iostream>
// #include <visp3/core/vpConfig.h>
// #if defined( VISP_HAVE_FRANKA )
// #include <visp3/robot/vpRobotFranka.h>
// int
// main( int argc, char **argv )
// {
//   std::string robot_ip = "172.16.0.2";
//   try
//   {
//     vpRobotFranka robot;
//     robot.connect( robot_ip );
//     std::cout << "WARNING: This example will move the robot! "
//               << "Please make sure to have the user stop button at hand!" << std::endl
//               << "Press Enter to continue..." << std::endl;
//     std::cin.ignore();

//     vpColVector initial_q = { -0.2113152342,  0.211263765, 0.5191193356, -1.486009226,
//                               -0.08664920681, 1.736819775, -0.4036056139 };

//     /*
//      * Get the actual joint position
//      */

//     vpColVector ee_q( 7, 0 );
//     robot.setRobotState( vpRobot::STATE_POSITION_CONTROL );
//     robot.getPosition( vpRobot::JOINT_STATE, ee_q );
//     std::cout << "EE joint position: " << ee_q.t() << std::endl;

//     /*
//      * Move to current position
//      */

//     // vpColVector q(7, 0);
//     // q = ee_q;
//     // robot.setRobotState(vpRobot::STATE_POSITION_CONTROL);
//     // std::cout << "Move to joint position: " << initial_q.t() << std::endl;
//     // robot.setPosition(vpRobot::JOINT_STATE, initial_q);

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
//   catch ( const vpException &e )
//   {
//     std::cout << "ViSP exception: " << e.what() << std::endl;
//     return EXIT_FAILURE;
//   }
//   catch ( const franka::NetworkException &e )
//   {
//     std::cout << "Franka network exception: " << e.what() << std::endl;
//     std::cout << "Check if you are connected to the Franka robot"
//               << " or if you specified the right IP using --ip command"
//               << " line option set by default to 192.168.1.1. " << std::endl;
//     return EXIT_FAILURE;
//   }
//   catch ( const std::exception &e )
//   {
//     std::cout << "Franka exception: " << e.what() << std::endl;
//     return EXIT_FAILURE;
//   }
//   std::cout << "The end" << std::endl;
//   return EXIT_SUCCESS;
// }
// #else
// int
// main()
// {
//   std::cout << "ViSP is not build with libfranka..." << std::endl;
// }
// #endif










#include <art_publisher/body.h>
#include <art_publisher/marker.h>
#include <iostream>
#include <math.h>
#include <ros/ros.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <visp3/robot/vpRobotFranka.h>
#include <visp3/visual_features/vpFeatureThetaU.h>
#include <visp3/visual_features/vpFeatureTranslation.h>
#include <visp3/vs/vpServo.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpPlot.h>
#include <tf/transform_listener.h>

#define CONFIG_FILE_PATH "./src/visp_ros/tutorial/franka/real-robot/config/configFile.config"
art_publisher::body local_pos;
vpColVector v_c( 6 );
using namespace std;

volatile sig_atomic_t flag = 0;
double x_pos;
double y_pos;
double z_pos;
double x_or;
double y_or;
double z_or;
double w_or;
bool has_converged             = false;
bool stop_program              = false;
double convergence_threshold_t = 0.1, convergence_threshold_tu = 2.0;

double eedmo_t_x;
double eedmo_t_y;
double eedmo_t_z;
double eedmo_r_x;
double eedmo_r_y;
double eedmo_r_z;
double eedmo_r_w;

double wMl_zero_t_x;
double wMl_zero_t_y;
double wMl_zero_t_z;
double wMl_zero_r_x;
double wMl_zero_r_y;
double wMl_zero_r_z;
double wMl_zero_r_w;

double wmee_ini_x;
double wmee_ini_y;
double wmee_ini_z;
double t_constraint_x_forward;
double t_constraint_x_backward;
double t_constraint_y_left;
double t_constraint_y_right;
double t_constraint_z_up;
double t_constraint_z_down;

vpHomogeneousMatrix wMo_ini;
vpPlot *plotter = nullptr;

void
setFromConfigFile()
{

  std::ifstream in( CONFIG_FILE_PATH );

  if ( !in.is_open() )
  {
    cout << "Cannot open configuration file from: " << CONFIG_FILE_PATH << strerror( errno ) << endl;
  }

  std::string param;
  double value;

  while ( !in.eof() )
  {
    in >> param;
    in >> value;

    if ( param == "EEDMO_TRANSLATION_X" )
    {
      eedmo_t_x = value;
    }
    if ( param == "EEDMO_TRANSLATION_Y" )
    {
      eedmo_t_y = value;
    }
    if ( param == "EEDMO_TRANSLATION_Z" )
    {
      eedmo_t_z = value;
    }
    if ( param == "EEDMO_QUARTENION_X" )
    {
      eedmo_r_x = value;
    }
    if ( param == "EEDMO_QUARTENION_Y" )
    {
      eedmo_r_y = value;
    }
    if ( param == "EEDMO_QUARTENION_Z" )
    {
      eedmo_r_z = value;
    }
    if ( param == "EEDMO_QUARTENION_W" )
    {
      eedmo_r_w = value;
    }

    if ( param == "WMLZERO_TRANSLATION_X" )
    {
      wMl_zero_t_x = value;
    }
    if ( param == "WMLZERO_TRANSLATION_Y" )
    {
      wMl_zero_t_y = value;
    }
    if ( param == "WMLZERO_TRANSLATION_Z" )
    {
      wMl_zero_t_z = value;
    }
    if ( param == "WMLZERO_QUARTENION_X" )
    {
      wMl_zero_r_x = value;
    }
    if ( param == "WMLZERO_QUARTENION_Y" )
    {
      wMl_zero_r_y = value;
    }
    if ( param == "WMLZERO_QUARTENION_Z" )
    {
      wMl_zero_r_z = value;
    }
    if ( param == "WMLZERO_QUARTENION_W" )
    {
      wMl_zero_r_w = value;
    }

    if ( param == "STARTING_WMEE_X" )
    {
      wmee_ini_x = value;
    }
    if ( param == "STARTING_WMEE_Y" )
    {
      wmee_ini_y = value;
    }
    if ( param == "STARTING_WMEE_Z" )
    {
      wmee_ini_z = value;
    }

    if ( param == "BOUNDING_BOX_X_forward" )
    {
      t_constraint_x_forward = value;
    }
    if ( param == "BOUNDING_BOX_X_backward" )
    {
      t_constraint_x_backward = value;
    }
    if ( param == "BOUNDING_BOX_Y_left" )
    {
      t_constraint_y_left = value;
    }
    if ( param == "BOUNDING_BOX_Y_right" )
    {
      t_constraint_y_right = value;
    }
    if ( param == "BOUNDING_BOX_Z_up" )
    {
      t_constraint_z_up = value;
    }
    if ( param == "BOUNDING_BOX_Z_down" )
    {
      t_constraint_z_down = value;
    }

    double wMoo_ini_tx;
    double wMoo_ini_ty;
    double wMoo_ini_tz;
    double wMoo_ini_rx;
    double wMoo_ini_ry;
    double wMoo_ini_rz;
    double wMoo_ini_rw;

    if ( param == "WMO_TX" )
    {
      wMoo_ini_tx = value;
    }
    if ( param == "WMO_TY" )
    {
      wMoo_ini_ty = value;
    }
    if ( param == "WMO_TZ" )
    {
      wMoo_ini_tz = value;
    }
    if ( param == "WMO_RX" )
    {
      wMoo_ini_rx = value;
    }
    if ( param == "WMO_RY" )
    {
      wMoo_ini_ry = value;
    }
    if ( param == "WMO_RZ" )
    {
      wMoo_ini_rz = value;
    }

    if ( param == "WMO_RW" )
    {
      wMoo_ini_rw = value;
    }

    wMo_ini.buildFrom( vpTranslationVector( wMoo_ini_tx, wMoo_ini_ty, wMoo_ini_tz ), vpQuaternionVector( wMoo_ini_rx, wMoo_ini_ry, wMoo_ini_rz, wMoo_ini_rw ) );
  }

  in.close();
}

void
artCallback( const art_publisher::body::ConstPtr &msg )
{
  x_pos = msg->bodies[0].pose.position.x;
  y_pos = msg->bodies[0].pose.position.y;
  z_pos = msg->bodies[0].pose.position.z;
  x_or  = msg->bodies[0].pose.orientation.x;
  y_or  = msg->bodies[0].pose.orientation.y;
  z_or  = msg->bodies[0].pose.orientation.z;
  w_or  = msg->bodies[0].pose.orientation.w;
}

void
my_function( int sig )
{
  flag = 1;
}

float
clip( float n, float lower, float upper )
{
  return std::max( lower, std::min( n, upper ) );
}

int
isARTNormalized()
{
  vpQuaternionVector currentQuartenion( x_or, y_or, z_or, w_or );
  if ( currentQuartenion.sumSquare() < 0.98 )
  {
    return 0;
  }
  else
  {
    return 1;
  }
}

void outputDataFiles(){
  time_t rawtime;
  struct tm * timeinfo;
  char buffer[80];
  time (&rawtime);
  timeinfo = localtime(&rawtime);
  strftime(buffer,sizeof(buffer),"%d-%m-%Y %H:%M",timeinfo);
  std::string str(buffer);
  std::string str_error;
  std::string str_vc;
  str_error = str+"-error.dat";
  str_vc=str+"-velocity.dat";
  plotter->saveData(0, str_error);
  plotter->saveData(1, str_vc);
}

int
main( int argc, char **argv )
{
  std::string opt_robot_ip = "172.16.0.2";
  vpRobotFranka robot;
  v_c = 0;

  try
  {
    ros::init( argc, argv, "control_node" );
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe( "ARTBody", 1000, artCallback );
    ros::Rate loop_rate( 10 );
    tf::TransformListener listener;
    robot.connect( "172.16.0.2" );
    cout << "Connected " << endl;
    setFromConfigFile();

    vpColVector ee_state( 6 );
    robot.setRobotState( vpRobot::STATE_VELOCITY_CONTROL );
    robot.getPosition( vpRobot::END_EFFECTOR_FRAME, ee_state );

    vpHomogeneousMatrix eedMee;
    vpHomogeneousMatrix eedMo( vpTranslationVector( eedmo_t_x, eedmo_t_y, eedmo_t_z ),
                               vpQuaternionVector( eedmo_r_x, eedmo_r_y, eedmo_r_z, eedmo_r_w ) );
    // - Translation: [-0.251, 0.046, 0.004]
    // - Rotation: in Quaternion [0.361, 0.891, -0.075, -0.266]
    //             in RPY (radian) [-2.774, -0.433, 2.289]
    //             in RPY (degree) [-158.918, -24.791, 131.170]

    vpHomogeneousMatrix wMo( vpTranslationVector( x_pos, y_pos, z_pos ), vpQuaternionVector( x_or, y_or, z_or, w_or ) );
    vpHomogeneousMatrix l_zeroMee( vpTranslationVector( ee_state[0], ee_state[1], ee_state[2] ),
                                   vpThetaUVector( ee_state[3], ee_state[4], ee_state[5] ) );
    vpHomogeneousMatrix wMl_zero( vpTranslationVector( wMl_zero_t_x, wMl_zero_t_y, wMl_zero_t_z ),
                                  vpQuaternionVector( wMl_zero_r_x, wMl_zero_r_y, wMl_zero_r_z, wMl_zero_r_w ) );
    // - Translation: [0.218, -0.164, 0.169]
    // - Rotation: in Quaternion [-0.049, -0.013, 0.722, 0.690]
    //             in RPY (radian) [-0.087, 0.052, 1.615]
    //             in RPY (degree) [-4.985, 2.979, 92.533]

    vpHomogeneousMatrix wMee = wMl_zero * l_zeroMee;
    eedMee                   = eedMo * wMo.inverse() * wMee;
    vpServo task;
    task.setServo( vpServo::EYEINHAND_CAMERA );
    task.setInteractionMatrixType( vpServo::CURRENT );

    // Features
    vpFeatureTranslation t( vpFeatureTranslation::cdMc );
    vpFeatureThetaU tu( vpFeatureThetaU::cdRc );
    t.buildFrom( eedMee );
    tu.buildFrom( eedMee );
    vpFeatureTranslation td( vpFeatureTranslation::cdMc );
    vpFeatureThetaU tud( vpFeatureThetaU::cdRc );
    task.addFeature( t, td );
    task.addFeature( tu, tud );
    task.setLambda( 0.65 );

    signal( SIGINT, my_function );
    int iter_plot   = 0;

    plotter = new vpPlot( 2, static_cast< int >( 250 * 2 ), 500, static_cast< int >( 0 ) + 80, 10,
                          "Real time curves plotter" );
    plotter->setTitle( 0, "Visual features error" );
    plotter->setTitle( 1, "Velocities" );
    plotter->initGraph( 0, 6 );
    plotter->initGraph( 1, 6 );
    plotter->setLegend( 0, 0, "error_feat_tx" );
    plotter->setLegend( 0, 1, "error_feat_ty" );
    plotter->setLegend( 0, 2, "error_feat_tz" );
    plotter->setLegend( 0, 3, "error_feat_theta_ux" );
    plotter->setLegend( 0, 4, "error_feat_theta_uy" );
    plotter->setLegend( 0, 5, "error_feat_theta_uz" );
    plotter->setLegend( 1, 0, "vc_x" );
    plotter->setLegend( 1, 1, "vc_y" );
    plotter->setLegend( 1, 2, "vc_z" );
    plotter->setLegend( 1, 3, "wc_x" );
    plotter->setLegend( 1, 4, "wc_y" );
    plotter->setLegend( 1, 5, "wc_z" );

    while ( !stop_program )
    {
      // while(0){
      ros::Subscriber sub = n.subscribe( "ARTBody", 1000, artCallback );
      // Update Matrixes
      vpColVector ee;
      robot.getPosition( vpRobot::END_EFFECTOR_FRAME, ee );
      vpHomogeneousMatrix wMo( vpTranslationVector( x_pos, y_pos, z_pos ),
                               vpQuaternionVector( x_or, y_or, z_or, w_or ) );
      vpHomogeneousMatrix l_zeroMee( vpTranslationVector( ee[0], ee[1], ee[2] ),
                                     vpThetaUVector( ee[3], ee[4], ee[5] ) );
      vpHomogeneousMatrix wMee = wMl_zero * l_zeroMee;

      // Update visual features
      eedMee = eedMo * wMo.inverse() * wMee;
      t.buildFrom( eedMee );
      tu.buildFrom( eedMee );

      v_c = task.computeControlLaw();

      double maxVel = robot.getMaxTranslationVelocity();
      double maxRot = robot.getMaxRotationVelocity();

      float dt = 0.1;

      float nt_x = v_c[0] * dt;
      float nt_y = v_c[1] * dt;
      float nt_z = v_c[2] * dt;
      float nr_x = v_c[3] * dt;
      float nr_y = v_c[4] * dt;
      float nr_z = v_c[5] * dt;

      // calibration matrices
      // starting point ee wrt world = 0.4623542903, 0.136219078, 0.3078160891

      // cout << "Ee rotation: "    << wMee.getRotationMatrix() << endl;
      // cout << "Ee translation: " << wMee.getTranslationVector() << endl;

      float x_forward  = wmee_ini_x + t_constraint_x_forward;
      float x_backward = wmee_ini_x - t_constraint_x_backward;
      float y_left     = wmee_ini_y + t_constraint_y_left;
      float y_right    = wmee_ini_y - t_constraint_y_right;
      float z_up       = wmee_ini_z + t_constraint_z_up;
      float z_down     = wmee_ini_z - t_constraint_z_down;

      z_up    = clip( z_up, 0.0, 0.58 );
      y_left  = clip( y_left, 0.0, 0.49 );
      z_down = clip( z_down, 0.0, 0.37);

      vpColVector np_ee( 4 );
      np_ee[0] = nt_x;
      np_ee[1] = nt_y;
      np_ee[2] = nt_z;
      np_ee[3] = 1;

      // translation next wrt world
      vpColVector next_world = wMee * np_ee;

      // rotation next wrt world
      vpHomogeneousMatrix eeMee_next( vpTranslationVector( nt_x, nt_y, nt_z ), vpThetaUVector( nr_x, nr_y, nr_z ) );
      vpHomogeneousMatrix wMee_next = wMee * eeMee_next;

      vpMatrix identityMatrix( 3, 3 );
      identityMatrix.setIdentity( 1.0 );
      vpMatrix rotMatrixnext = wMee_next.getRotationMatrix();
      float thresholdXMax    = 9e-13;
      float thresholdXMin    = 3e-12;
      float thresholdYMax    = 5e-9;
      float thresholdYMin    = 1e-13;
      float thresholdZMax    = 3e-13;
      float thresholdZMin    = 6e-13;


      tf::Quaternion q_tf(x_or, y_or, z_or, w_or);
      tfScalar yaw_tf, pitch_tf, roll_tf;
      tf::Matrix3x3 mat(q_tf);
      mat.getEulerYPR(yaw_tf,pitch_tf,roll_tf);
      vpHomogeneousMatrix o_iniMo = wMo.inverse() * wMo_ini;
      vpThetaUVector o_iniMo_tu = o_iniMo.getThetaUVector();
      double error_tu_object = vpMath::deg( sqrt( o_iniMo_tu.sumSquare() ) );

      if (isARTNormalized()){
        cout << "error tu object 0: " << vpMath::deg(o_iniMo_tu[0]) << " , " << vpMath::deg(o_iniMo_tu[1])<< " , " << vpMath::deg(o_iniMo_tu[2]) <<endl;

      }
      // cout << "current roll: "<< roll_tf<<endl;
      // cout << "init pitch: " << wMoo_ini_p<<endl;
      // cout << "current pitch: "<< pitch_tf<<endl;

      // negative  is right and positive is left
      //cout << "result pitch: " << fabs(vpMath::deg(wMoo_ini_p)-vpMath::deg(pitch_tf))<<endl;

      if (  
          x_backward <= next_world[0] && next_world[0] <= x_forward 
          && y_right <= next_world[1] && next_world[1] <= y_left 
          && z_down  <= next_world[2] && next_world[2] <= z_up
          &&!( isnan( v_c[0] ) ) && !( isnan( v_c[1] ) ) 
          &&!( isnan( v_c[2] ) ) && !( isnan( v_c[3] ) ) 
          &&!( isnan( v_c[4] ) ) && !( isnan( v_c[5] ) )
          && isARTNormalized() 
          && !(has_converged)
          )
      {
        // here set velocity
        v_c[0] = clip( v_c[0], -0.1, 0.1 );
        v_c[1] = clip( v_c[1], -0.1, 0.1 );
        v_c[2] = clip( v_c[2], -0.1, 0.1 );
        v_c[3] = clip( v_c[3], -0.1, 0.1 );
        v_c[4] = clip( v_c[4], -0.1, 0.1 );
        v_c[5] = clip( v_c[5], -0.1, 0.1 );
        //robot.setVelocity( vpRobot::END_EFFECTOR_FRAME, v_c );
        //cout << "Robot moving v_c : " << v_c << endl;
      }
      else
      {
        // cout << "Constraint violated " << endl;
        // cout << "v_c" << v_c << endl;
        if ( !isARTNormalized() )
        {
          cout << "Constraint violated. Art out of range" << endl;
        }

        // translation constraints
        if ( next_world[0] < x_backward )
        {
          cout << "X backward violated: translation x" << next_world[0] << endl;
        }
        if ( next_world[0] > x_forward )
        {
          cout << "X forward violated: translation x" << next_world[0] << endl;
        }
        if ( next_world[1] < y_right )
        {
          cout << "Y right violated: translation y" << next_world[1] << endl;
        }
        if ( next_world[1] > y_left )
        {
          cout << "Y left violated: translation y" << next_world[1] << endl;
        }
        if ( next_world[2] < z_down )
        {
          cout << "Z down violated: translation z" << next_world[2] << endl;
        }
        if ( next_world[2] > z_up )
        {
          cout << "Z up violated: translation z" << next_world[2] << endl;
        }
        if(has_converged)
        {
          cout << "Servo task has converged" << endl;
        }

        v_c = 0;
        robot.setVelocity( vpRobot::END_EFFECTOR_FRAME, v_c );
      }

      // cout << "print x  " << next_world[0] << endl;
      // cout << "print y  " << next_world[1] << endl;
      // cout << "print z  " << next_world[2] << endl;

      vpTranslationVector eed_t_ee = eedMee.getTranslationVector();
      vpThetaUVector eed_tu_ee     = eedMee.getThetaUVector();
      double error_tr              = sqrt( eed_t_ee.sumSquare() );
      double error_tu              = vpMath::deg( sqrt( eed_tu_ee.sumSquare() ) );

      // cout << "error_t: " << error_tr << endl;
      // cout << "error_tu: " << error_tu << endl;

      plotter->plot( 0, iter_plot, task.getError() );
      plotter->plot( 1, iter_plot, v_c );
      iter_plot++;

      if ( error_tr < convergence_threshold_t && error_tu < convergence_threshold_tu )
      {
        has_converged = true;
      } else {
        has_converged = false; 
      }

      if ( flag )
      {
        printf( "\n Signal caught!\n" );
        printf( "\n default action it not termination!\n" );
        flag = 0;
        v_c  = 0;
        robot.setRobotState( vpRobot::STATE_STOP );
        stop_program = true;
      }
      
      ros::spinOnce();
      loop_rate.sleep();
    } // while(1)
    outputDataFiles();
    if ( plotter != nullptr )
    {
      delete plotter;
      plotter = nullptr;
    }

  }
  catch ( const vpException &e )
  {
    cout << "ViSP exception: " << e.what() << endl;
    cout << "Stop the robot " << endl;
    robot.setRobotState( vpRobot::STATE_STOP );
    outputDataFiles();
    return EXIT_FAILURE;
  }
  catch ( const franka::NetworkException &e )
  {
    cout << "Franka network exception: " << e.what() << endl;
    cout << "Check if you are connected to the Franka robot"
         << " or if you specified the right IP using --ip command line option set by default to 192.168.1.1. " << endl;
    outputDataFiles();
    return EXIT_FAILURE;
  }
  catch ( const std::exception &e )
  {
    cout << "Franka exception: " << e.what() << endl;
    outputDataFiles();
    return EXIT_FAILURE;
  }

  outputDataFiles();

  return 0;
}
