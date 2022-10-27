#include <iostream>
#include <visp_ros/vpROSGrabber.h>

#include <visp3/core/vpCameraParameters.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpPlot.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/visual_features/vpFeatureThetaU.h>
#include <visp3/visual_features/vpFeatureTranslation.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>
#include <visp_ros/vpROSRobotFrankaCoppeliasim.h>

#include <array>
#include <limits>
#include <visp3/core/vpQuaternionVector.h>

#include <ros/ros.h>
#include <art_publisher/marker.h>
#include <art_publisher/body.h>

#include <signal.h> 

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
typedef std::array<float, 3> float3;
float3 getEulerAngles(float q0, float q1, float q2, float q3)
{
    return
    {
        atan2(2 * (q0*q1 + q2*q3), 1 - 2 * (q1*q1 + q2*q2)),
        asin( 2 * (q0*q2 - q3*q1)),
        atan2(2 * (q0*q3 + q1*q2), 1 - 2 * (q2*q2 + q3*q3))
    };
}

art_publisher::body local_pos;
vpColVector v_c( 6);

volatile sig_atomic_t flag = 0;
double x_pos;
double y_pos;
double z_pos;
double x_or;
double y_or;
double z_or;
double w_or;
bool has_converged = false;
bool stop_program  = false;
double convergence_threshold_t = 0.01, convergence_threshold_tu = 0.2;

void artCallback (const art_publisher::body::ConstPtr &msg)
{
  x_pos = msg->bodies[2].pose.position.x/1000;
  y_pos = msg->bodies[2].pose.position.y/1000;
  z_pos = msg->bodies[2].pose.position.z/1000;
  x_or  = msg->bodies[2].pose.orientation.x; 
  y_or  = msg->bodies[2].pose.orientation.y; 
  z_or  = msg->bodies[2].pose.orientation.z; 
  w_or  = msg->bodies[2].pose.orientation.w;

  // std::cout << "ART x" << x_pos << std::endl;
  // std::cout << "ART y" << y_pos << std::endl;
  // std::cout << "ART z" << z_pos << std::endl;
  // std::cout << "ART rot x" << x_or  << std::endl;
  // std::cout << "ART rot y" << y_or  << std::endl;
  // std::cout << "ART rot z" << z_or  << std::endl;
  // std::cout << "ART rot w" << w_or  << std::endl;
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
  double opt_tagSize             = 0.08;
  bool display_tag               = true;
  int opt_quad_decimate          = 2;
  bool opt_verbose               = true;
  bool opt_plot                  = true;
  bool opt_adaptive_gain         = true;
  bool opt_task_sequencing       = false;
  double convergence_threshold_t = 0.05;
  double convergence_threshold_tu = vpMath::rad( 5 );
  bool opt_coppeliasim_sync_mode = true;
  bool canBeComputed = true;

  vpColVector dotq(6) ;

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
  
  vpColVector joint_state;
  vpColVector ee_state;
  vpColVector camera_state;
  vpColVector reference_frame;

  vpServo task;

  // robot.setRobotFrame( vpRobot::ARTICULAR_FRAME );

  // robot.getPosition( vpRobot::END_EFFECTOR_FRAME, ee_state );
  // // Desired pose used to compute the desired features
  vpHomogeneousMatrix eedMo( vpTranslationVector( 0, 0.0, 0.3), vpQuaternionVector(0.7071068, 0.7071068, 0, 0 )) ;
  // vpHomogeneousMatrix fMc( vpTranslationVector(0.575, 0.000000046, 0.685) , vpQuaternionVector(-0.5, 0.5, -0.5, 0.5));
  // vpHomogeneousMatrix fMee( vpTranslationVector( ee_state[0], ee_state[1], ee_state[2] ), vpThetaUVector(ee_state[3], ee_state[4], ee_state[5]) );

  vpHomogeneousMatrix cMf, cMee, eedMee, cMo, oMo;
  // vpMatrix fJe;

  // cMf = fMc.inverse();
  // cMee = cMf*fMee;

  // vpImage< unsigned char > I;
  // vpROSGrabber g;
  // g.setImageTopic( "/coppeliasim/franka/camera/image" );
  // g.setCameraInfoTopic( "/coppeliasim/franka/camera/camera_info" );
  // g.open( argc, argv );
  // g.acquire( I );

  // std::cout << "Image size: " << I.getWidth() << " x " << I.getHeight() << std::endl;
  // vpCameraParameters cam;

  // g.getCameraInfo( cam );
  // std::cout << cam << std::endl;
  // vpDisplayOpenCV dc( I, 10, 10, "Color image" );

  vpDetectorAprilTag::vpAprilTagFamily tagFamily                  = vpDetectorAprilTag::TAG_36h11;
  vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS;
  vpDetectorAprilTag detector( tagFamily );
  detector.setAprilTagPoseEstimationMethod( poseEstimationMethod );
  detector.setDisplayTag( display_tag );
  detector.setAprilTagQuadDecimate( opt_quad_decimate );

  eedMee = eedMo * cMo.inverse()*cMee;

  vpFeatureTranslation t( vpFeatureTranslation::cdMc );
  vpFeatureThetaU tu( vpFeatureThetaU::cdRc );

  //initialization of the current feature translation s= (tx,ty,tz) and theta_u= theta_ux, theta_uy and theta_uz
  t.buildFrom( eedMee );
  tu.buildFrom( eedMee );

  vpFeatureTranslation td(  vpFeatureTranslation::cdMc );
  vpFeatureThetaU tud( vpFeatureThetaU::cdRc );

  task.addFeature( t, td );
  task.addFeature( tu, tud );

  task.setServo( vpServo::EYEINHAND_CAMERA );
  task.setInteractionMatrixType( vpServo::CURRENT );

  robot.setCoppeliasimSyncMode( opt_coppeliasim_sync_mode );

  if ( opt_adaptive_gain )
  {
    std::cout << "Enable adaptive gain" << std::endl;
    vpAdaptiveGain lambda( 4, 1.2, 25 ); // lambda(0)=4, lambda(oo)=1.2 and lambda'(0)=25
    task.setLambda( lambda );
  }
  else
  {
    task.setLambda( 2.5 );
  }

  vpPlot *plotter = nullptr;

  // if ( opt_plot )
  // {
  //   plotter = new vpPlot( 2, static_cast< int >( 250 * 2 ), 500, static_cast< int >( I.getWidth() ) + 80, 10,
  //                         "Real time curves plotter" );
  //   plotter->setTitle( 0, "Visual features error" );
  //   plotter->setTitle( 1, "Velocities" );
  //   plotter->initGraph( 0, 6 );
  //   plotter->initGraph( 1, 6 );
  //   plotter->setLegend( 0, 0, "error_feat_tx" );
  //   plotter->setLegend( 0, 1, "error_feat_ty" );
  //   plotter->setLegend( 0, 2, "error_feat_tz" );
  //   plotter->setLegend( 0, 3, "error_feat_theta_ux" );
  //   plotter->setLegend( 0, 4, "error_feat_theta_uy" );
  //   plotter->setLegend( 0, 5, "error_feat_theta_uz" );
  //   plotter->setLegend( 1, 0, "vc_x" );
  //   plotter->setLegend( 1, 1, "vc_y" );
  //   plotter->setLegend( 1, 2, "vc_z" );
  //   plotter->setLegend( 1, 3, "wc_x" );
  //   plotter->setLegend( 1, 4, "wc_y" );
  //   plotter->setLegend( 1, 5, "wc_z" );
  // }

  bool final_quit                           = false;
  bool has_converged                        = false;
  bool send_velocities                      = false;
  bool servo_started                        = false;
  // std::vector< vpImagePoint > *traj_corners = nullptr; 

  double sim_time            = robot.getCoppeliasimSimulationTime();
  double sim_time_prev       = sim_time;
  double sim_time_init_servo = sim_time;
  double sim_time_img        = sim_time;
  vpColVector v_c( 6);

  while ( !stop_program )
  {
      robot.setRobotState( vpRobot::STATE_VELOCITY_CONTROL );

      v_c[0]=0.00;
      v_c[1]=0.00;
      v_c[2]=0.00;
      v_c[3]=0.00;
      v_c[4]=0.00;
      v_c[5]=0.00;

    robot.setVelocity( vpRobot::END_EFFECTOR_FRAME, v_c);
    // vpDisplay::flush( I );
    if(flag) {
    printf("\n Signal caught!\n");
    printf("\n default action it not termination!\n");
    flag = 0;
    v_c = 0;
    // robot.setRobotState(vpRobot::STATE_STOP);
    stop_program = true;
    }
    // robot.wait( sim_time, 0.020 );
  }

  if ( opt_plot && plotter != nullptr )
  {
    delete plotter;
    plotter = nullptr;
  }

  robot.coppeliasimStopSimulation();

}