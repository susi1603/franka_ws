#include <iostream>

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
#include <stdio.h>
#include <stdlib.h>
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
  bool opt_verbose               = false;
  bool opt_plot                  = false;
  bool opt_adaptive_gain         = false;
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
  ros::NodeHandle n;
  // ros::NodeHandlePtr n = boost::make_shared< ros::NodeHandle >();
  ros::Subscriber sub = n.subscribe("ARTBody", 1000, artCallback);
  ros::Rate loop_rate( 1000 );
  // ros::spinOnce();

  vpROSRobotFrankaCoppeliasim robot;
  robot.setVerbose( opt_verbose );
  robot.connect();

  std::cout << "Coppeliasim sync mode enabled: " << ( opt_coppeliasim_sync_mode ? "yes" : "no" ) << std::endl;
  robot.coppeliasimStopSimulation(); 
  robot.setCoppeliasimSyncMode( false );
  robot.coppeliasimStartSimulation();
  
  
  vpColVector ee_state(6);
  robot.setRobotState( vpRobot::STATE_VELOCITY_CONTROL);
  robot.getPosition( vpRobot::END_EFFECTOR_FRAME, ee_state);

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

  signal(SIGINT, my_function); 

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

  // vpPlot *plotter = nullptr;

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
  std::vector< vpImagePoint > *traj_corners = nullptr; 

  double sim_time            = robot.getCoppeliasimSimulationTime();
  double sim_time_prev       = sim_time;
  double sim_time_init_servo = sim_time;
  double sim_time_img        = sim_time;
  vpColVector v_c( 6);
  while ( !final_quit )
  {
    // Update Matrixes
    vpColVector ee;
    // ros::Subscriber sub = n.subscribe("ARTBody", 1000, artCallback);
    robot.getPosition( vpRobot::END_EFFECTOR_FRAME, ee);
    vpHomogeneousMatrix wMo(vpTranslationVector(x_pos, y_pos, z_pos), vpQuaternionVector(x_or, y_or, z_or, w_or));
    vpHomogeneousMatrix l_zeroMee(vpTranslationVector( ee[0], ee[1], ee[2] ), vpThetaUVector(ee[3], ee[4], ee[5]));
    vpHomogeneousMatrix wMl_zero(vpTranslationVector(0.478,-0.312,0.080), vpQuaternionVector(-0.012, -0.001, 0.724, 0.690));
    vpHomogeneousMatrix wMee = wMl_zero * l_zeroMee;

    // Update visual features
    eedMee = eedMo * wMo.inverse() * wMee;
    // std::cout << "eedMee" << eedMee << std::endl;
    t.buildFrom( eedMee );
    tu.buildFrom( eedMee);

    std::cout << "eedMee" << eedMee.getTranslationVector().t() << std::endl;
    v_c = task.computeControlLaw();

    // if ( opt_plot )
    // {
    //   plotter->plot( 0, static_cast< double >( sim_time ), task.getError() );
    //   plotter->plot( 1, static_cast< double >( sim_time ), v_c );
    // }
    std::stringstream ss;
    ss << "Loop time [s]: " << std::round( ( sim_time - sim_time_prev ) * 1000. ) / 1000.;
    ss << " Simulation time [s]: " << sim_time;
    sim_time_prev = sim_time;

    vpMouseButton::vpMouseButtonType button;
    float dt = 0.15;

    float nt_x = v_c[0]*dt;
    float nt_y = v_c[1]*dt;
    float nt_z = v_c[2]*dt;
    float nr_x = v_c[3]*dt;
    float nr_y = v_c[4]*dt;
    float nr_z = v_c[5]*dt;

    // calibration matrices 
    // starting point ee wrt world = 0.4623542903, 0.136219078, 0.3078160891


    // std::cout << "Ee rotation: "    << wMee.getRotationMatrix() << std::endl;
    // std::cout << "Ee translation: " << wMee.getTranslationVector() << std::endl;

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

    //ROTATION CONSTRAINING

    vpMatrix rotx_Mmin(3,3);
    //trial 2
    rotx_Mmin[0][0] = -0.05515376604;
    rotx_Mmin[0][1] = 0.9985703389;
    rotx_Mmin[0][2] = -0.02564210731;
    rotx_Mmin[1][0] = 0.6700535155;
    rotx_Mmin[1][1] = 0.05603082482;
    rotx_Mmin[1][2] = 0.740763802;
    rotx_Mmin[2][0] = 0.7408296199;
    rotx_Mmin[2][1] = 0.02366436658;
    rotx_Mmin[2][2] = -0.6719030059;

    vpMatrix rotx_Mmax(3,3);
    //trial 1
    rotx_Mmax[0][0] = -0.03264962268;
    rotx_Mmax[0][1] = 0.9981487898;
    rotx_Mmax[0][2] = 0.058950596;
    rotx_Mmax[1][0] = 0.7302647459;
    rotx_Mmax[1][1] = 0.0640933086;
    rotx_Mmax[1][2] = -0.6807698773;
    rotx_Mmax[2][0] = -0.6830004248;
    rotx_Mmax[2][1] = 0.02081389973;
    rotx_Mmax[2][2] = -0.7306978709;

    vpMatrix roty_Mmin(3,3);
    // trial 4
    roty_Mmin[0][0] = -0.05422519287;
    roty_Mmin[0][1] = 0.7886053749;
    roty_Mmin[0][2] = -0.6131911353;
    roty_Mmin[1][0] = 0.9986801102;
    roty_Mmin[1][1] = 0.02851245909;
    roty_Mmin[1][2] = -0.05164546792;
    roty_Mmin[2][0] = -0.0232345247;
    roty_Mmin[2][1] = -0.6149233933;
    roty_Mmin[2][2] = -0.7887785205;

    vpMatrix roty_Mmax(3,3);
    // trial 3
    roty_Mmax[0][0] = 0.0362131;
    roty_Mmax[0][1] = 0.677744;
    roty_Mmax[0][2] = 0.734979;
    roty_Mmax[1][0] = 0.998942;
    roty_Mmax[1][1] = 0.0053165;
    roty_Mmax[1][2] = -0.0541212;
    roty_Mmax[2][0] = -0.0405708;
    roty_Mmax[2][1] = 0.735851;
    roty_Mmax[2][2] = -0.676549;

    vpMatrix rotz_Mmax(3,3);
    // trial 5
    rotz_Mmax[0][0] = 0.8396981127;
    rotz_Mmax[0][1] = 0.5423784022;
    rotz_Mmax[0][2] = -0.03968533226;
    rotz_Mmax[1][0] = 0.5409763683;
    rotz_Mmax[1][1] = -0.840534353;
    rotz_Mmax[1][2] = -0.04109437449;
    rotz_Mmax[2][0] = -0.05562216932;
    rotz_Mmax[2][1] = 0.01303255507;
    rotz_Mmax[2][2] = -0.9987885182;

    vpMatrix rotz_Mmin(3,3);
    // trial 6
    rotz_Mmin[0][0] = -0.7868229316;
    rotz_Mmin[0][1] = 0.6162155591;
    rotz_Mmin[0][2] = -0.04505814334;
    rotz_Mmin[1][0] = 0.617346887;
    rotz_Mmin[1][1] = 0.7870500566;
    rotz_Mmin[1][2] = -0.01664952816;
    rotz_Mmin[2][0] = 0.02519270983;
    rotz_Mmin[2][1] = -0.04089951639;
    rotz_Mmin[2][2] = -0.9992670985;

    vpMatrix identityMatrix(3,3);
    identityMatrix.setIdentity(1.0);
    vpMatrix rotMatrixnext = wMee_next.getRotationMatrix();
    float thresholdXMax = 9e-13;
    float thresholdXMin = 3e-12;
    float thresholdYMax = 5e-9;
    float thresholdYMin = 1e-13;
    float thresholdZMax = 3e-13;
    float thresholdZMin = 6e-13;
    
    // // Rotation Calibration
    // std::cout << "wMee rotation matrix: "<< std::endl;
    // std::cout << wMee.getRotationMatrix()[0][0] << std::endl;
    // std::cout << wMee.getRotationMatrix()[0][1] << std::endl;
    // std::cout << wMee.getRotationMatrix()[0][2] << std::endl;
    // std::cout << wMee.getRotationMatrix()[1][0] << std::endl;
    // std::cout << wMee.getRotationMatrix()[1][1] << std::endl;
    // std::cout << wMee.getRotationMatrix()[1][2] << std::endl;
    // std::cout << wMee.getRotationMatrix()[2][0] << std::endl;
    // std::cout << wMee.getRotationMatrix()[2][1] << std::endl;
    // std::cout << wMee.getRotationMatrix()[2][2] << std::endl;

    if (
        // x_backward<=next_world[0] && next_world[0]<=x_forward
        // && y_right<=next_world[1] && next_world[1]<=y_left
        // && z_down<=next_world[2] && next_world[2]<=z_up

        // // && std::fabs((rotx_Mmin*rotMatrixnext.inverseByLU()-identityMatrix).det())>=thresholdXMin
        // // && std::fabs((rotx_Mmax*rotMatrixnext.inverseByLU()-identityMatrix).det())>=thresholdXMax
        // // && std::fabs((roty_Mmin*rotMatrixnext.inverseByLU()-identityMatrix).det())>=thresholdYMin
        // // // && std::fabs((roty_Mmax*rotMatrixnext.inverseByLU()-identityMatrix).det())>=thresholdYMax
        // // && std::fabs((rotz_Mmin*rotMatrixnext.inverseByLU()-identityMatrix).det())>=thresholdZMin
        // // && std::fabs((rotz_Mmax*rotMatrixnext.inverseByLU()-identityMatrix).det())>=thresholdZMax

        // && !(isnan(v_c[0]))
        // && !(isnan(v_c[1]))
        // && !(isnan(v_c[2]))
        // && !(isnan(v_c[3]))
        // && !(isnan(v_c[4]))
        // && !(isnan(v_c[5]))

        // && isARTNormalized()
        isARTNormalized()
        ) 
    {
      // here set velocity
      v_c[0] = clip(v_c[0], -0.1 ,0.1);
      v_c[1] = clip(v_c[1], -0.1 ,0.1);
      v_c[2] = clip(v_c[2], -0.1 ,0.1);
      v_c[3] = clip(v_c[3], -0.1 ,0.1);
      v_c[4] = clip(v_c[4], -0.1 ,0.1);
      v_c[5] = clip(v_c[5], -0.1 ,0.1);

      robot.setVelocity(vpRobot::END_EFFECTOR_FRAME, v_c);
      std::cout << "v_c" << v_c << std::endl;
    }
    else
    {
      std::cout << "Constraint violated " << std::endl;
      // std::cout << "v_c" << v_c << std::endl;
      if (!isARTNormalized()){
        std::cout << "Art out of range "<< std::endl;
      }
      // rotation matrix constraints
      if (std::fabs((rotx_Mmin*rotMatrixnext.inverseByLU()-identityMatrix).det())<thresholdXMin){
        float res = std::fabs((rotx_Mmin*rotMatrixnext.inverseByLU()-identityMatrix).det());
        std::cout << "constraints rotx_Mmin "<< res << std::endl;
      }
      if (std::fabs((rotx_Mmax*rotMatrixnext.inverseByLU()-identityMatrix).det())<thresholdXMax){
        float res = std::fabs((rotx_Mmax*rotMatrixnext.inverseByLU()-identityMatrix).det());
        std::cout << "constraints rotx_Mmax "<< res << std::endl;
      }

      if (std::fabs((roty_Mmin*rotMatrixnext.inverseByLU()-identityMatrix).det())<thresholdYMin){
        float res = std::fabs((roty_Mmin*rotMatrixnext.inverseByLU()-identityMatrix).det());
        std::cout << "constraints roty_Mmin "<< res << std::endl;
      }
      if (std::fabs((roty_Mmax*rotMatrixnext.inverseByLU()-identityMatrix).det())<thresholdYMax){
        float res = std::fabs((roty_Mmax*rotMatrixnext.inverseByLU()-identityMatrix).det());
        std::cout << "constraints roty_Mmax "<< res << std::endl;
      }

      if (std::fabs((rotz_Mmin*rotMatrixnext.inverseByLU()-identityMatrix).det())<thresholdZMin){
        float res = std::fabs((rotz_Mmin*rotMatrixnext.inverseByLU()-identityMatrix).det());
        std::cout << "constraints rotz_Mmin "<< res << std::endl;
      }
      if (std::fabs((rotz_Mmax*rotMatrixnext.inverseByLU()-identityMatrix).det())<thresholdZMax){
        float res = std::fabs((rotz_Mmax*rotMatrixnext.inverseByLU()-identityMatrix).det());
        std::cout << "constraints rotz_Mmax "<< res << std::endl;
      }

      // translation constraints
      if (next_world[0]<x_backward || x_forward<next_world[0]){
        std::cout << "translation x "<< next_world[0] << std::endl;
      }
      if (next_world[1]<y_right || y_left<next_world[1]){
        std::cout << "translation y "<< next_world[1] << std::endl;
      }
      if (next_world[2]<z_down || z_up<next_world[2]){
        std::cout << "translation z "<< next_world[2] << std::endl;
      }

      v_c = 0;
      robot.setVelocity(vpRobot::END_EFFECTOR_FRAME, v_c);
    }

    vpTranslationVector cd_t_c = eedMee.getTranslationVector();
    vpThetaUVector cd_tu_c     = eedMee.getThetaUVector();
    double error_tr            = sqrt( cd_t_c.sumSquare() );
    double error_tu            = vpMath::deg( sqrt( cd_tu_c.sumSquare() ) );

    // std::cout << "error_t: " << error_tr << std::endl;
    // std::cout << "error_tu: " << error_tu << std::endl;

    if ( error_tr < convergence_threshold_t && error_tu < convergence_threshold_tu )
    {
      has_converged = true;
      std::cout << "Servo task has converged" << std::endl;
    }

    if(flag){
        printf("\n Signal caught!\n");
        printf("\n default action it not termination!\n");
        flag = 0;
        v_c = 0;
        final_quit = true;
        robot.coppeliasimStopSimulation();
    }

    ros::spinOnce();
    loop_rate.sleep();
    robot.wait( sim_time, 0.020 );
  }

  // if ( opt_plot && plotter != nullptr )
  // {
  //   delete plotter;
  //   plotter = nullptr;
  // }

  robot.coppeliasimStopSimulation();

}