//! \example tutorial-franka-coppeliasim-pbvs-apriltag.cpp

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

#include <visp_ros/vpROSGrabber.h>
#include <visp_ros/vpROSRobotFrankaCoppeliasim.h>

#include <array>
#include <limits>
#include <visp3/core/vpQuaternionVector.h>

void
display_point_trajectory( const vpImage< unsigned char > &I, const std::vector< vpImagePoint > &vip,
                          std::vector< vpImagePoint > *traj_vip )
{
  for ( size_t i = 0; i < vip.size(); i++ )
  {
    if ( traj_vip[i].size() )
    {
      // Add the point only if distance with the previous > 1 pixel
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
  bool opt_coppeliasim_sync_mode = false;
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
  robot.coppeliasimStopSimulation(); 
  robot.setCoppeliasimSyncMode( false );
  robot.coppeliasimStartSimulation();
  
  vpServo task;
  vpColVector joint_state;
  vpColVector ee_state;
  vpColVector camera_state;
  vpColVector reference_frame;
  
  robot.setRobotState( vpRobot::STATE_VELOCITY_CONTROL );
  task.setServo( vpServo::EYEINHAND_CAMERA);
  task.setInteractionMatrixType( vpServo::CURRENT);
  // robot.setRobotFrame( vpRobot::ARTICULAR_FRAME );

  robot.getPosition( vpRobot::END_EFFECTOR_FRAME, ee_state );
  // Desired pose used to compute the desired features
  vpHomogeneousMatrix eedMo( vpTranslationVector( 0, 0.0, 0.3), vpQuaternionVector(0.7071068, 0.7071068, 0, 0 )) ;
  vpHomogeneousMatrix fMc( vpTranslationVector(0.575, 0.000000046, 0.685) , vpQuaternionVector(-0.5, 0.5, -0.5, 0.5));
  vpHomogeneousMatrix fMee( vpTranslationVector( ee_state[0], ee_state[1], ee_state[2] ), vpThetaUVector(ee_state[3], ee_state[4], ee_state[5]) );

  vpHomogeneousMatrix cMf, cMee, eedMee, cMo, oMo;
  vpMatrix fJe;

  cMf = fMc.inverse();
  cMee = cMf*fMee;

  vpImage< unsigned char > I;
  vpROSGrabber g;
  g.setImageTopic( "/coppeliasim/franka/camera/image" );
  g.setCameraInfoTopic( "/coppeliasim/franka/camera/camera_info" );
  g.open( argc, argv );
  g.acquire( I );

  std::cout << "Image size: " << I.getWidth() << " x " << I.getHeight() << std::endl;
  vpCameraParameters cam;

  g.getCameraInfo( cam );
  std::cout << cam << std::endl;
  vpDisplayOpenCV dc( I, 10, 10, "Color image" );

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

  if ( opt_plot )
  {
    plotter = new vpPlot( 2, static_cast< int >( 250 * 2 ), 500, static_cast< int >( I.getWidth() ) + 80, 10,
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
  }

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
    sim_time = robot.getCoppeliasimSimulationTime();
    g.acquire( I, sim_time_img );

    vpDisplay::display( I );
    std::vector< vpHomogeneousMatrix > cMo_vec;
    detector.detect( I, opt_tagSize, cam, cMo_vec );

    {
      // Display the trajectory of the points used as features
      std::stringstream ss;
      ss << "Left click to " << ( send_velocities ? "stop the robot" : "servo the robot" )
          << ", right click to quit.";
      vpDisplay::displayText( I, 20, 20, ss.str(), vpColor::red );
    }

    vpColVector uee_state;
    robot.getPosition( vpRobot::END_EFFECTOR_FRAME, uee_state );
    vpHomogeneousMatrix ufMee( vpTranslationVector( uee_state[0], uee_state[1], uee_state[2] ), vpThetaUVector(uee_state[3], uee_state[4], uee_state[5]) );

    cMf = fMc.inverse();
    cMee = cMf*ufMee;

    task.set_cVf(cMf);
    
    if ( cMo_vec.size() == 1 )
    {
      cMo = cMo_vec[0];
      static bool first_time = true;
      
      if ( first_time )
      {
        std::vector< vpHomogeneousMatrix > v_oMo( 2 ), v_cdMc( 2 );
        // .buildFrom ( tx, ty, tz tux, tuy, tuz);
        v_oMo[1].buildFrom( 0, 0, 0, 0, 0, M_PI );
        for ( size_t i = 0; i < 2; i++ )
        {
          v_cdMc[i] = eedMo * v_oMo[i] * cMo.inverse();
        }
        if ( std::fabs( v_cdMc[0].getThetaUVector().getTheta() ) <
              std::fabs( v_cdMc[1].getThetaUVector().getTheta() ) )
        {
          oMo = v_oMo[0];
        }
        else
        {
          std::cout << "Desired frame modified to avoid PI rotation of the camera" << std::endl;
          oMo = v_oMo[1]; // Introduce PI rotation
        }
      }

      vpHomogeneousMatrix oMc;
      oMc = cMo.inverse();

      //update visual features
      eedMee = eedMo*oMc*cMee;
      t.buildFrom( eedMee );
      tu.buildFrom( eedMee);

      // robot.get_fJe(fJe);
      // task.set_fJe(fJe);
      v_c = task.computeControlLaw();

      vpDisplay::displayFrame( I, eedMo * oMo, cam, opt_tagSize / 1.5, vpColor::yellow, 2 );
      vpDisplay::displayFrame( I, cMo, cam, opt_tagSize / 2, vpColor::none, 3 );

      // Get tag corners
      std::vector< vpImagePoint > corners = detector.getPolygon( 0 );

      // Get the tag cog corresponding to the projection of the tag frame in the image
      corners.push_back( detector.getCog( 0 ) );

      // Display the trajectory of the points
      if ( first_time )
      {
        traj_corners = new std::vector< vpImagePoint >[corners.size()];
      }

      display_point_trajectory( I, corners, traj_corners );


      if ( opt_plot )
      {
        plotter->plot( 0, static_cast< double >( sim_time ), task.getError() );
        plotter->plot( 1, static_cast< double >( sim_time ), v_c );
      }

      vpTranslationVector cd_t_c = eedMee.getTranslationVector();
      vpThetaUVector cd_tu_c     = eedMee.getThetaUVector();
      double error_tr            = sqrt( cd_t_c.sumSquare() );
      double error_tu            = vpMath::deg( sqrt( cd_tu_c.sumSquare() ) );

      std::stringstream ss;
      ss << "error_t: " << error_tr;
      vpDisplay::displayText( I, 20, static_cast< int >( I.getWidth() ) - 150, ss.str(), vpColor::red );
      ss.str( "" );
      ss << "error_tu: " << error_tu;
      vpDisplay::displayText( I, 40, static_cast< int >( I.getWidth() ) - 150, ss.str(), vpColor::red );

      if ( !has_converged && error_tr < convergence_threshold_t && error_tu < convergence_threshold_tu )
      {
        has_converged = true;
        std::cout << "Servo task has converged"
                  << "\n";
        vpDisplay::displayText( I, 100, 20, "Servo task has converged", vpColor::red );
      }
      if ( first_time )
      {
        first_time = false;
      }

    } // end if (cMo_vec.size() == 1)
    else
    {
      v_c = 0;
    }

    if ( !send_velocities )
    {
      v_c = 0;
    }

    float dt = 0.020;
    vpColVector np_ee(4);
    vpColVector ee_state_current;

    robot.getPosition( vpRobot::END_EFFECTOR_FRAME, ee_state_current );
    vpHomogeneousMatrix fMee_current( vpTranslationVector( ee_state_current[0], ee_state_current[1], ee_state_current[2] ), vpThetaUVector(ee_state_current[3], ee_state_current[4], ee_state_current[5]) );

    float nt_x = v_c[0]*dt;
    float nt_y = v_c[1]*dt;
    float nt_z = v_c[2]*dt;
    float nr_x = v_c[3]*dt;
    float nr_y = v_c[4]*dt;
    float nr_z = v_c[5]*dt;

    float distance_x = 0.15;
    float distance_y = 0.125;
    float distance_z = 0.30;

    float x_forward = 0.5 + distance_x;
    float x_backward = 0.5 - distance_x;
    float y_left = 0.00000046 + distance_y;
    float y_right = 0.00000046 - distance_y;
    float z_up = 0.685 + distance_z;
    float z_down = 0.685 - distance_z;

    np_ee[0] = nt_x;
    np_ee[1] = nt_y;
    np_ee[2] = nt_z;
    np_ee[3] = 1;

    vpHomogeneousMatrix eeMee_next( vpTranslationVector( nt_x, nt_y, nt_z ), vpThetaUVector(nr_x, nr_y, nr_z) );
    vpColVector next_world = fMee_current * np_ee; 
    vpHomogeneousMatrix fMee_next = fMee_current * eeMee_next;

    //ROTATION CONSTRAINING 

    float rotx_min = -3.14;
    float rotx_max = 3.14;
    float roty_min = -0.9;
    float roty_max = 0.9;
    float rotz_min = -0.1;
    float rotz_max = 1.85;

    vpMatrix rotx_Mmin(3,3); 
    //bjval= -0.785
    rotx_Mmin[0][0] = -0.00243294;
    rotx_Mmin[0][1] = 0.000854336;
    rotx_Mmin[0][2] = 0.999997;
    rotx_Mmin[1][0] = 0.705922;
    rotx_Mmin[1][1] = -0.708286;
    rotx_Mmin[1][2] = 0.00232259;
    rotx_Mmin[2][0] = 0.708285;
    rotx_Mmin[2][1] = 0.705925;
    rotx_Mmin[2][2] = 0.00112013;

    vpMatrix rotx_Mmax(3,3);
    //bjval = 0.785
    rotx_Mmax[0][0] = -0.00118685;
    rotx_Mmax[0][1] = 0.00356238;
    rotx_Mmax[0][2] = 0.999993;
    rotx_Mmax[1][0] = 0.709997;
    rotx_Mmax[1][1] = 0.704203;
    rotx_Mmax[1][2] = -0.00166599;
    rotx_Mmax[2][0] = -0.704204;
    rotx_Mmax[2][1] = 0.70999;
    rotx_Mmax[2][2] = -0.00336506;

    vpMatrix roty_Mmin(3,3);
    //ryval = 0.05
    roty_Mmin[0][0] = 0.0081468;
    roty_Mmin[0][1] = -0.0398222;
    roty_Mmin[0][2] = 0.999174;
    roty_Mmin[1][0] = 0.999964;
    roty_Mmin[1][1] = -0.00200541;
    roty_Mmin[1][2] = -0.00823317;
    roty_Mmin[2][0] = 0.00233162;
    roty_Mmin[2][1] = 0.999205;
    roty_Mmin[2][2] = 0.0398044;

    vpMatrix roty_Mmax(3,3);
    //ryval = -0.523
    roty_Mmax[0][0] = 0.00561701;
    roty_Mmax[0][1] = 0.494465;
    roty_Mmax[0][2] = 0.86918;
    roty_Mmax[1][0] = 0.999984;
    roty_Mmax[1][1] = -0.00329659;
    roty_Mmax[1][2] = -0.00458693;
    roty_Mmax[2][0] = 0.000597257;
    roty_Mmax[2][1] = 0.869192;
    roty_Mmax[2][2] = -0.494475;

    vpMatrix rotz_Mmin(3,3);
    //rzval = -0.261799
    rotz_Mmin[0][0] = 0.257212;
    rotz_Mmin[0][1] = 0.000796591;
    rotz_Mmin[0][2] = 0.966355;
    rotz_Mmin[1][0] = 0.966351;
    rotz_Mmin[1][1] = -0.00314788;
    rotz_Mmin[1][2] = -0.257208;
    rotz_Mmin[2][0] = 0.00283708;
    rotz_Mmin[2][1] = 0.999995;
    rotz_Mmin[2][2] = -0.00157946;

    vpMatrix rotz_Mmax(3,3);
    //rzval=0.261799
    rotz_Mmax[0][0] = -0.254934;
    rotz_Mmax[0][1] = -0.00162303;
    rotz_Mmax[0][2] = 0.966957;
    rotz_Mmax[1][0] = 0.966954;
    rotz_Mmax[1][1] = -0.00337087;
    rotz_Mmax[1][2] = 0.254927;
    rotz_Mmax[2][0] = 0.00284573;
    rotz_Mmax[2][1] = 0.999993;
    rotz_Mmax[2][2] = 0.00242874;

    // vpQuaternionVector qr;
    // fMee_next.extract(qr);
    // float3 eulerAngles = getEulerAngles(qr[3],qr[0],qr[1],qr[2]);

    vpMatrix rotationMax(3,3); 
    vpMatrix identityMatrix(3,3);
    identityMatrix.setIdentity(1.0);
    vpMatrix rotMatrixnext = fMee_next.getRotationMatrix();
    // float threshold = 7.42e-10;
    float threshold = 3.52e-8;

    // float constraint = (rotMatrixnext*rotationMax.inverse()-identityMatrix).det();


    //to get the max rotation matrixes
    std::cout << "fMee_current: "<< std::endl;
    std::cout << fMee_current.getRotationMatrix()[0][0] << std::endl;
    std::cout << fMee_current.getRotationMatrix()[0][1] << std::endl;
    std::cout << fMee_current.getRotationMatrix()[0][2] << std::endl;
    std::cout << fMee_current.getRotationMatrix()[1][0] << std::endl;
    std::cout << fMee_current.getRotationMatrix()[1][1] << std::endl;
    std::cout << fMee_current.getRotationMatrix()[1][2] << std::endl;
    std::cout << fMee_current.getRotationMatrix()[2][0] << std::endl;
    std::cout << fMee_current.getRotationMatrix()[2][1] << std::endl;
    std::cout << fMee_current.getRotationMatrix()[2][2] << std::endl;
   
    if (
        x_backward<=next_world[0] && next_world[0]<=x_forward
        && y_right<=next_world[1] && next_world[1]<=y_left
        && z_down<=next_world[2] && next_world[2]<=z_up

        && std::fabs((rotx_Mmin*rotMatrixnext.inverseByLU()-identityMatrix).det())>=threshold
        && std::fabs((rotx_Mmax*rotMatrixnext.inverseByLU()-identityMatrix).det())>=threshold
        && std::fabs((roty_Mmin*rotMatrixnext.inverseByLU()-identityMatrix).det())>=threshold
        && std::fabs((roty_Mmax*rotMatrixnext.inverseByLU()-identityMatrix).det())>=threshold
        && std::fabs((rotz_Mmin*rotMatrixnext.inverseByLU()-identityMatrix).det())>=threshold
        && std::fabs((rotz_Mmax*rotMatrixnext.inverseByLU()-identityMatrix).det())>=threshold

        // && rotx_min<=eulerAngles[0] && eulerAngles[0]<=rotx_max
        // && roty_min<=eulerAngles[1] && eulerAngles[1]<=roty_max
        // && rotz_min<=eulerAngles[2] && eulerAngles[2]<=rotz_max
        ) 
    {

      robot.setVelocity( vpRobot::END_EFFECTOR_FRAME, v_c);

    }
    else
    {
      std::cout << "Constraint violated " << std::endl;

      // euler rotation constraints
      // if (eulerAngles[0]<=rotx_min || rotx_max<=eulerAngles[0]){
      //   std::cout << "rotation x "<< eulerAngles[0] << std::endl;
      // }
      // if (eulerAngles[1]<=roty_min || roty_max<=eulerAngles[1]){
      //   std::cout << "rotation y "<< eulerAngles[1] << std::endl;
      // }
      // if (eulerAngles[2]<=rotz_min || rotz_max<=eulerAngles[2]){
      //   std::cout << "rotation z "<< eulerAngles[2] << std::endl;
      // }

      // rotation matrix constraints
      if (std::fabs((rotx_Mmin*rotMatrixnext.inverseByLU()-identityMatrix).det())<threshold){
        float res = std::fabs((rotx_Mmin*rotMatrixnext.inverseByLU()-identityMatrix).det());
        std::cout << "constraints rotx_Mmin "<< res << std::endl;
      }
      if (std::fabs((rotx_Mmax*rotMatrixnext.inverseByLU()-identityMatrix).det())<threshold){
        float res = std::fabs((rotx_Mmax*rotMatrixnext.inverseByLU()-identityMatrix).det());
        std::cout << "constraints rotx_Mmax "<< res << std::endl;
      }

      if (std::fabs((roty_Mmin*rotMatrixnext.inverseByLU()-identityMatrix).det())<threshold){
        float res = std::fabs((roty_Mmin*rotMatrixnext.inverseByLU()-identityMatrix).det());
        std::cout << "constraints roty_Mmin "<< res << std::endl;
      }
      if (std::fabs((roty_Mmax*rotMatrixnext.inverseByLU()-identityMatrix).det())<threshold){
        float res = std::fabs((roty_Mmax*rotMatrixnext.inverseByLU()-identityMatrix).det());
        std::cout << "constraints roty_Mmax "<< res << std::endl;
      }

      if (std::fabs((rotz_Mmin*rotMatrixnext.inverseByLU()-identityMatrix).det())<threshold){
        float res = std::fabs((rotz_Mmin*rotMatrixnext.inverseByLU()-identityMatrix).det());
        std::cout << "constraints rotz_Mmin "<< res << std::endl;
      }
      if (std::fabs((rotz_Mmax*rotMatrixnext.inverseByLU()-identityMatrix).det())<threshold){
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
      robot.setVelocity( vpRobot::END_EFFECTOR_FRAME, v_c);
    }

    std::stringstream ss;
    ss << "Loop time [s]: " << std::round( ( sim_time - sim_time_prev ) * 1000. ) / 1000.;
    ss << " Simulation time [s]: " << sim_time;
    sim_time_prev = sim_time;
    vpDisplay::displayText( I, 40, 20, ss.str(), vpColor::red );

    vpMouseButton::vpMouseButtonType button;
    if ( vpDisplay::getClick( I, button, false ) )
    {
      switch ( button )
      {
      case vpMouseButton::button1:
        send_velocities = !send_velocities;
        break;

      case vpMouseButton::button3:
        final_quit = true;
        v_c        = 0;
        
        break;

      default:
        break;
      }
    }

    vpDisplay::flush( I );
    robot.wait( sim_time, 0.020 ); // Slow down the loop to simulate a camera at 50 Hz
  }

  if ( opt_plot && plotter != nullptr )
  {
    delete plotter;
    plotter = nullptr;
  }

  robot.coppeliasimStopSimulation();

  if ( !final_quit )
  {
    while ( !final_quit )
    {
      g.acquire( I );
      vpDisplay::display( I );

      vpDisplay::displayText( I, 20, 20, "Click to quit the program.", vpColor::red );
      vpDisplay::displayText( I, 40, 20, "Visual servo converged.", vpColor::red );

      if ( vpDisplay::getClick( I, false ) )
      {
        final_quit = true;
      }

      vpDisplay::flush( I );
    }
  }
  if ( traj_corners )
  {
    delete[] traj_corners;
  }

}