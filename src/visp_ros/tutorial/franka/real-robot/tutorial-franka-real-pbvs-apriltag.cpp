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
double convergence_threshold_t = 0.01, convergence_threshold_tu = 0.2;

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
  }

  in.close();

  // Log the loaded configuration
  cout << "eedMo translation (x,y,z) and quartenion rotation (x,y,z,w):" << endl;
  cout << eedmo_t_x << endl;
  cout << eedmo_t_y << endl;
  cout << eedmo_t_z << endl;
  cout << eedmo_r_x << endl;
  cout << eedmo_r_y << endl;
  cout << eedmo_r_z << endl;
  cout << eedmo_r_w << endl;

  cout << "wMl_zero translation (x,y,z) and quartenion rotation (x,y,z,w):" << endl;
  cout << wMl_zero_t_x << endl;
  cout << wMl_zero_t_y << endl;
  cout << wMl_zero_t_z << endl;
  cout << wMl_zero_r_x << endl;
  cout << wMl_zero_r_y << endl;
  cout << wMl_zero_r_z << endl;
  cout << wMl_zero_r_w << endl;

  cout << "starting position end-effector with respect to world frame translation (x,y,z): " << endl;
  cout << wmee_ini_x << endl;
  cout << wmee_ini_y << endl;
  cout << wmee_ini_z << endl;

  cout << "distances constraints along z: " << endl;
  // cout << t_constraint_x << endl;
  // cout << t_constraint_y << endl;
  cout << t_constraint_z_up << endl;
  cout << t_constraint_z_down << endl;
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
    task.setLambda( 0.55 );

    signal( SIGINT, my_function );

    vpPlot *plotter = nullptr;
    int iter_plot   = 0;

    plotter = new vpPlot( 2, static_cast< int >( 250 * 2 ), 500, static_cast< int >( 640 ) + 80, 10,
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

      clip( z_up, 0.0, 0.593325 );

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

      // ROTATION CONSTRAINING
      vpMatrix rotx_Mmin( 3, 3 );
      // trial 2
      rotx_Mmin[0][0] = -0.05515376604;
      rotx_Mmin[0][1] = 0.9985703389;
      rotx_Mmin[0][2] = -0.02564210731;
      rotx_Mmin[1][0] = 0.6700535155;
      rotx_Mmin[1][1] = 0.05603082482;
      rotx_Mmin[1][2] = 0.740763802;
      rotx_Mmin[2][0] = 0.7408296199;
      rotx_Mmin[2][1] = 0.02366436658;
      rotx_Mmin[2][2] = -0.6719030059;

      vpMatrix rotx_Mmax( 3, 3 );
      // trial 1
      rotx_Mmax[0][0] = -0.03264962268;
      rotx_Mmax[0][1] = 0.9981487898;
      rotx_Mmax[0][2] = 0.058950596;
      rotx_Mmax[1][0] = 0.7302647459;
      rotx_Mmax[1][1] = 0.0640933086;
      rotx_Mmax[1][2] = -0.6807698773;
      rotx_Mmax[2][0] = -0.6830004248;
      rotx_Mmax[2][1] = 0.02081389973;
      rotx_Mmax[2][2] = -0.7306978709;

      vpMatrix roty_Mmin( 3, 3 );
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

      vpMatrix roty_Mmax( 3, 3 );
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

      vpMatrix rotz_Mmax( 3, 3 );
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

      vpMatrix rotz_Mmin( 3, 3 );
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

      vpMatrix identityMatrix( 3, 3 );
      identityMatrix.setIdentity( 1.0 );
      vpMatrix rotMatrixnext = wMee_next.getRotationMatrix();
      float thresholdXMax    = 9e-13;
      float thresholdXMin    = 3e-12;
      float thresholdYMax    = 5e-9;
      float thresholdYMin    = 1e-13;
      float thresholdZMax    = 3e-13;
      float thresholdZMin    = 6e-13;

      // // Rotation Calibration
      // cout << "wMee rotation matrix: "<< endl;
      // cout << wMee.getRotationMatrix()[0][0] << endl;
      // cout << wMee.getRotationMatrix()[0][1] << endl;
      // cout << wMee.getRotationMatrix()[0][2] << endl;
      // cout << wMee.getRotationMatrix()[1][0] << endl;
      // cout << wMee.getRotationMatrix()[1][1] << endl;
      // cout << wMee.getRotationMatrix()[1][2] << endl;
      // cout << wMee.getRotationMatrix()[2][0] << endl;
      // cout << wMee.getRotationMatrix()[2][1] << endl;
      // cout << wMee.getRotationMatrix()[2][2] << endl;

      if (  
          x_backward <= next_world[0] && next_world[0] <= x_forward 
          && y_right <= next_world[1] && next_world[1] <= y_left 
          && z_down  <= next_world[2] && next_world[2] <= z_up
          // && std::fabs((rotx_Mmin*rotMatrixnext.inverseByLU()-identityMatrix).det())>=thresholdXMin
          // && std::fabs((rotx_Mmax*rotMatrixnext.inverseByLU()-identityMatrix).det())>=thresholdXMax
          // && std::fabs((roty_Mmin*rotMatrixnext.inverseByLU()-identityMatrix).det())>=thresholdYMin
          // && std::fabs((roty_Mmax*rotMatrixnext.inverseByLU()-identityMatrix).det())>=thresholdYMax
          // && std::fabs((rotz_Mmin*rotMatrixnext.inverseByLU()-identityMatrix).det())>=thresholdZMin
          // && std::fabs((rotz_Mmax*rotMatrixnext.inverseByLU()-identityMatrix).det())>=thresholdZMax
          &&!( isnan( v_c[0] ) ) && !( isnan( v_c[1] ) ) 
          &&!( isnan( v_c[2] ) ) && !( isnan( v_c[3] ) ) 
          &&!( isnan( v_c[4] ) ) && !( isnan( v_c[5] ) )
          && isARTNormalized() 
          )
      {
        // here set velocity
        v_c[0] = clip( v_c[0], -0.1, 0.1 );
        v_c[1] = clip( v_c[1], -0.1, 0.1 );
        v_c[2] = clip( v_c[2], -0.1, 0.1 );
        v_c[3] = clip( v_c[3], -0.1, 0.1 );
        v_c[4] = clip( v_c[4], -0.1, 0.1 );
        v_c[5] = clip( v_c[5], -0.1, 0.1 );

        robot.setVelocity( vpRobot::END_EFFECTOR_FRAME, v_c );
        cout << "Robot moving v_c : " << v_c << endl;
      }
      else
      {
        // cout << "Constraint violated " << endl;
        // cout << "v_c" << v_c << endl;
        if ( !isARTNormalized() )
        {
          cout << "Constraint violated. Art out of range" << endl;
        }
        // rotation matrix constraints
        if ( fabs( ( rotx_Mmin * rotMatrixnext.inverseByLU() - identityMatrix ).det() ) < thresholdXMin )
        {
          float res = fabs( ( rotx_Mmin * rotMatrixnext.inverseByLU() - identityMatrix ).det() );
          cout << "Constraint violated rotx_Mmin :" << res << endl;
        }
        if ( fabs( ( rotx_Mmax * rotMatrixnext.inverseByLU() - identityMatrix ).det() ) < thresholdXMax )
        {
          float res = fabs( ( rotx_Mmax * rotMatrixnext.inverseByLU() - identityMatrix ).det() );
          cout << "Constraint violated rotx_Mmax " << res << endl;
        }

        if ( fabs( ( roty_Mmin * rotMatrixnext.inverseByLU() - identityMatrix ).det() ) < thresholdYMin )
        {
          float res = fabs( ( roty_Mmin * rotMatrixnext.inverseByLU() - identityMatrix ).det() );
          cout << "Constraint violated roty_Mmin " << res << endl;
        }
        if ( fabs( ( roty_Mmax * rotMatrixnext.inverseByLU() - identityMatrix ).det() ) < thresholdYMax )
        {
          float res = fabs( ( roty_Mmax * rotMatrixnext.inverseByLU() - identityMatrix ).det() );
          cout << "Constraint violated roty_Mmax " << res << endl;
        }

        if ( fabs( ( rotz_Mmin * rotMatrixnext.inverseByLU() - identityMatrix ).det() ) < thresholdZMin )
        {
          float res = fabs( ( rotz_Mmin * rotMatrixnext.inverseByLU() - identityMatrix ).det() );
          cout << "Constraint violated rotz_Mmin " << res << endl;
        }
        if ( fabs( ( rotz_Mmax * rotMatrixnext.inverseByLU() - identityMatrix ).det() ) < thresholdZMax )
        {
          float res = fabs( ( rotz_Mmax * rotMatrixnext.inverseByLU() - identityMatrix ).det() );
          cout << "Constraint violated rotz_Mmax " << res << endl;
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

        v_c = 0;
        robot.setVelocity( vpRobot::END_EFFECTOR_FRAME, v_c );
      }


      vpTranslationVector eed_t_ee = eedMee.getTranslationVector();
      vpThetaUVector eed_tu_ee     = eedMee.getThetaUVector();
      double error_tr            = sqrt( eed_t_ee.sumSquare() );
      double error_tu            = vpMath::deg( sqrt( eed_tu_ee.sumSquare() ) );

      // cout << "error_t: " << error_tr << endl;
      // cout << "error_tu: " << error_tu << endl;

      plotter->plot( 0, iter_plot, task.getError() );
      plotter->plot( 1, iter_plot, v_c );
      iter_plot++;

      if ( error_tr < convergence_threshold_t && error_tu < convergence_threshold_tu )
      {
        has_converged = true;
        cout << "Servo task has converged" << endl;
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
    return EXIT_FAILURE;
  }
  catch ( const franka::NetworkException &e )
  {
    cout << "Franka network exception: " << e.what() << endl;
    cout << "Check if you are connected to the Franka robot"
         << " or if you specified the right IP using --ip command line option set by default to 192.168.1.1. " << endl;
    return EXIT_FAILURE;
  }
  catch ( const std::exception &e )
  {
    cout << "Franka exception: " << e.what() << endl;
    return EXIT_FAILURE;
  }

  return 0;
}
