#include <iostream>
#include <visp_ros/vpROSGrabber.h>

#include <visp3/core/vpCameraParameters.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpPlot.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>
#include <visp_ros/vpROSRobotFrankaCoppeliasim.h>
#include <visp3/visual_features/vpFeatureThetaU.h>
#include <visp3/visual_features/vpFeatureTranslation.h>

#include <array>
#include <limits>
#include <visp3/core/vpQuaternionVector.h>

#include <ros/ros.h>
#include <signal.h>
#include <tf/transform_listener.h>

vpColVector v_c( 6);

volatile sig_atomic_t flag = 0;
double x_pos = 0.0;
double y_pos = 0.0;
double z_pos = 0.0;
double x_or = 0.0;
double y_or = 0.0;
double z_or = 0.0;
double w_or = 0.0;
bool has_converged = false;
bool stop_program  = false;

double opt_tagSize             = 0.08;
bool display_tag               = true;
int opt_quad_decimate          = 2;
bool opt_verbose               = true;
bool opt_plot                  = false;
bool opt_adaptive_gain         = false;
bool opt_task_sequencing       = false;
double convergence_threshold_t = 0.05;
double convergence_threshold_tu = vpMath::rad( 5 );
bool opt_coppeliasim_sync_mode = false;

void
display_point_trajectory( const vpImage< unsigned char > &I, const std::vector< vpImagePoint > &vip,
                          std::vector< vpImagePoint > *traj_vip )
{
  for ( size_t i = 0; i < vip.size(); i++ )
  {
    if ( traj_vip[i].size() )
    {
      if ( vpImagePoint::distance( vip[i], traj_vip[i].back() ) > 1. )
      {
        traj_vip[i].push_back( vip[i] );
      }
    }
    else
    {
      traj_vip[i].push_back( vip[i] );
    }
  }
  for ( size_t i = 0; i < vip.size(); i++ )
  {
    for ( size_t j = 1; j < traj_vip[i].size(); j++ )
    {
      vpDisplay::displayLine( I, traj_vip[i][j - 1], traj_vip[i][j], vpColor::green, 2 );
    }
  }
}

void my_function(int sig){
  flag = 1;
}

float clip(float n, float lower, float upper) {
  return std::max(lower, std::min(n, upper));
}

int isARTNormalized(){
  vpQuaternionVector currentQuartenion(x_or, y_or, z_or, w_or);
  if (currentQuartenion.sumSquare() < 0.98){
     return 0;
  }
  else {
    return 1;
  }
}

int
main( int argc, char **argv )
{
  for ( int i = 1; i < argc; i++ )
  {
    if ( std::string( argv[i] ) == "--tag_size" && i + 1 < argc )
    {
      opt_tagSize = std::stod( argv[i + 1] );
    }
    else if ( std::string( argv[i] ) == "--verbose" || std::string( argv[i] ) == "-v" )
    {
      opt_verbose = true;
    }
    else if ( std::string( argv[i] ) == "--plot" )
    {
      opt_plot = true;
    }
    else if ( std::string( argv[i] ) == "--adaptive_gain" )
    {
      opt_adaptive_gain = true;
    }
    else if ( std::string( argv[i] ) == "--quad_decimate" && i + 1 < argc )
    {
      opt_quad_decimate = std::stoi( argv[i + 1] );
    }
    else if ( std::string( argv[i] ) == "--no-convergence-threshold" )
    {
      convergence_threshold_t  = 0.;
      convergence_threshold_tu = 0.;
    }
    else if ( std::string( argv[i] ) == "--enable-coppeliasim-sync-mode" )
    {
      opt_coppeliasim_sync_mode = true;
    }
    else if ( std::string( argv[i] ) == "--help" || std::string( argv[i] ) == "-h" )
    {
      std::cout << argv[0] << "[--tag_size <marker size in meter; default " << opt_tagSize << ">] "
                << "[--quad_decimate <decimation; default " << opt_quad_decimate << ">] "
                << "[--adaptive_gain] "
                << "[--plot] "
                << "[--task_sequencing] "
                << "[--no-convergence-threshold] "
                << "[--enable-coppeliasim-sync-mode] "
                << "[--verbose] [-v] "
                << "[--help] [-h]" << std::endl;
      return EXIT_SUCCESS;
    }
  }

  try
  {
    ros::init( argc, argv, "visp_ros" );
    ros::NodeHandlePtr n = boost::make_shared< ros::NodeHandle >();
    ros::Rate loop_rate( 1000 );
    ros::spinOnce();

    vpROSRobotFrankaCoppeliasim robot;
    robot.setVerbose( opt_verbose );
    robot.connect();

    std::cout << "Coppeliasim sync mode enabled: " << ( opt_coppeliasim_sync_mode ? "yes" : "no" ) << std::endl;
    robot.coppeliasimStopSimulation(); // Allows to reset simulation, moving the robot to initial position
    robot.setCoppeliasimSyncMode( false );
    robot.coppeliasimStartSimulation();
    
    vpColVector ee_state(6);
    robot.setRobotState( vpRobot::STATE_VELOCITY_CONTROL);
    robot.getPosition( vpRobot::END_EFFECTOR_FRAME, ee_state);

    std::cout << "ee_state " << ee_state.t() << std::endl;

    vpHomogeneousMatrix eedMee;
    vpHomogeneousMatrix eedMo(vpTranslationVector(-0.244, -0.027, 0.031), vpQuaternionVector(0.661, -0.054, 0.151, 0.733));
    vpHomogeneousMatrix wMo(vpTranslationVector(x_pos, y_pos, z_pos), vpQuaternionVector(x_or, y_or, z_or, w_or));
    vpHomogeneousMatrix l_zeroMee(vpTranslationVector( ee_state[0], ee_state[1], ee_state[2] ), vpThetaUVector(ee_state[3], ee_state[4], ee_state[5]));
    vpHomogeneousMatrix wMl_zero(vpTranslationVector(0.478,-0.312,0.080), vpQuaternionVector(-0.012, -0.001, 0.724, 0.690));
    vpHomogeneousMatrix wMee = wMl_zero * l_zeroMee;
    eedMee = eedMo * wMo.inverse() * wMee;
    vpServo task;
    task.setServo( vpServo::EYEINHAND_CAMERA);
    task.setInteractionMatrixType( vpServo::CURRENT);

    //Features 
    vpFeatureTranslation t( vpFeatureTranslation::cdMc );
    vpFeatureThetaU tu( vpFeatureThetaU::cdRc );
    t.buildFrom( eedMee );
    tu.buildFrom( eedMee );
    vpFeatureTranslation td(  vpFeatureTranslation::cdMc );
    vpFeatureThetaU tud( vpFeatureThetaU::cdRc );
    task.addFeature( t, td );
    task.addFeature( tu, tud );
    task.setLambda( 1 );
    double sim_time            = robot.getCoppeliasimSimulationTime();
    double sim_time_prev       = sim_time;
    double sim_time_init_servo = sim_time;
    double sim_time_img        = sim_time;
    signal(SIGINT, my_function); 
    robot.setCoppeliasimSyncMode( opt_coppeliasim_sync_mode );
    tf::TransformListener listener;

    while ( !stop_program )
    {
      sim_time = robot.getCoppeliasimSimulationTime();
      tf::StampedTransform transform;
      // try{
      //   listener.lookupTransform("/myworld", "/calib_tool_ee",
      //                            ros::Time(0), transform);
      // }
      // catch (tf::TransformException &ex) {
      //   ROS_ERROR("%s",ex.what());
      //   ros::Duration(1.0).sleep();
      //   continue;
      // }
        // Update Matrixes
      vpColVector ee;
      robot.getPosition( vpRobot::END_EFFECTOR_FRAME, ee);
      std::cout << "END EFFECTOR UPDATE " << ee.t() << std::endl;
      vpHomogeneousMatrix wMo(vpTranslationVector(x_pos, y_pos, z_pos), vpQuaternionVector(x_or, y_or, z_or, w_or));
      vpHomogeneousMatrix l_zeroMee(vpTranslationVector( ee[0], ee[1], ee[2] ), vpThetaUVector(ee[3], ee[4], ee[5]));
      vpHomogeneousMatrix wMl_zero(vpTranslationVector(0.478,-0.312,0.080), vpQuaternionVector(-0.012, -0.001, 0.724, 0.690));
      vpHomogeneousMatrix wMee = wMl_zero * l_zeroMee;

      // Update visual features
      eedMee = eedMo * wMo.inverse() * wMee;
      t.buildFrom( eedMee );
      tu.buildFrom( eedMee);
      v_c = task.computeControlLaw();

      float dt = 0.15;

      float nt_x = v_c[0]*dt;
      float nt_y = v_c[1]*dt;
      float nt_z = v_c[2]*dt;
      float nr_x = v_c[3]*dt;
      float nr_y = v_c[4]*dt;
      float nr_z = v_c[5]*dt;

      float distance_x = 0.20;
      float distance_y = 0.20;
      float distance_z = 0.20;

      float x_forward  = 0.4623542903 + distance_x;
      float x_backward = 0.4623542903 - distance_x;
      float y_left     = 0.136219078 + distance_y;
      float y_right    = 0.05;
      float z_up       = 0.3078160891 + distance_z;
      float z_down     = 0.15;

      vpColVector np_ee(4);
      np_ee[0] = nt_x;
      np_ee[1] = nt_y;
      np_ee[2] = nt_z;
      np_ee[3] = 1;
      
      // translation next wrt world
      vpColVector next_world = wMee * np_ee; 

      // rotation next wrt world
      vpHomogeneousMatrix eeMee_next( vpTranslationVector( nt_x, nt_y, nt_z), vpThetaUVector(nr_x, nr_y, nr_z) );
      vpHomogeneousMatrix wMee_next = wMee * eeMee_next;

      if (
          // x_backward<=next_world[0] && next_world[0]<=x_forward
          // && y_right<=next_world[1] && next_world[1]<=y_left
          // && z_down<=next_world[2] && next_world[2]<=z_up

           !(isnan(v_c[0]))
          && !(isnan(v_c[1]))
          && !(isnan(v_c[2]))
          && !(isnan(v_c[3]))
          && !(isnan(v_c[4]))
          && !(isnan(v_c[5]))

          // && isARTNormalized()
          ) 
      {
        // here set velocity
        v_c[0] = clip(v_c[0], -0.1 ,0.1);
        v_c[1] = clip(v_c[1], -0.1 ,0.1);
        v_c[2] = clip(v_c[2], -0.1 ,0.1);
        v_c[3] = clip(v_c[3], -0.1 ,0.1);
        v_c[4] = clip(v_c[4], -0.1 ,0.1);
        v_c[5] = clip(v_c[5], -0.1 ,0.1);

        // robot.setVelocity(vpRobot::END_EFFECTOR_FRAME, v_c);
        std::cout << "v_c" << v_c << std::endl;
      }
      else
      {
        std::cout << "Constraint violated " << std::endl;
        // // std::cout << "v_c" << v_c << std::endl;
        // if (!isARTNormalized()){
        //   std::cout << "Art out of range "<< std::endl;
        // }
        // // translation constraints
        // if (next_world[0]<x_backward || x_forward<next_world[0]){
        //   std::cout << "translation x "<< next_world[0] << std::endl;
        // }
        // if (next_world[1]<y_right || y_left<next_world[1]){
        //   std::cout << "translation y "<< next_world[1] << std::endl;
        // }
        // if (next_world[2]<z_down || z_up<next_world[2]){
        //   std::cout << "translation z "<< next_world[2] << std::endl;
        // }
        // v_c = 0;
        // robot.setVelocity(vpRobot::END_EFFECTOR_FRAME, v_c);
      }

      // vpTranslationVector cd_t_c = eedMee.getTranslationVector();
      // vpThetaUVector cd_tu_c     = eedMee.getThetaUVector();
      // double error_tr            = sqrt( cd_t_c.sumSquare() );
      // double error_tu            = vpMath::deg( sqrt( cd_tu_c.sumSquare() ) );

      // // std::cout << "error_t: " << error_tr << std::endl;
      // // std::cout << "error_tu: " << error_tu << std::endl;

      // if ( error_tr < convergence_threshold_t && error_tu < convergence_threshold_tu )
      // {
      //   has_converged = true;
      //   std::cout << "Servo task has converged" << std::endl;
      // }
      // sim_time_prev = sim_time;

      // if(flag){
      //     printf("\n Signal caught!\n");
      //     printf("\n default action it not termination!\n");
      //     flag = 0;
      //     v_c = 0;
      //     robot.coppeliasimStopSimulation();
      //     stop_program = true;
      // }
      std::cout << "before robot wait  " << std::endl;
      robot.wait( sim_time, 0.020 ); 
      loop_rate.sleep();
      std::cout << "after robot wait " << std::endl;
    }

    std::cout << "did it reach this point? " << std::endl;
    robot.coppeliasimStopSimulation();

  }
  catch ( const vpException &e )
  {
    std::cout << "ViSP exception: " << e.what() << std::endl;
    std::cout << "Stop the robot " << std::endl;
    return EXIT_FAILURE;
  }

  return 0;

}