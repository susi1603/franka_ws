#include <iostream>
#include <visp3/robot/vpRobotFranka.h>
#include <visp3/vs/vpServo.h>
#include <visp_ros/vpROSGrabber.h>
#include <art_publisher/marker.h>
#include <art_publisher/body.h>
#include <ros/ros.h>
#include <math.h>
#include <visp3/visual_features/vpFeatureThetaU.h>
#include <visp3/visual_features/vpFeatureTranslation.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h> 

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

int main(int argc, char **argv)
{
  std::string opt_robot_ip = "172.16.0.2";
  vpRobotFranka robot;
  v_c = 0;

  try {
  ros::init(argc, argv, "control_node");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("ARTBody", 1000, artCallback);
  ros::Rate loop_rate(10);
  robot.connect("172.16.0.2");
  std::cout << "connected " << std::endl;

  vpColVector ee_state(6);
  robot.setRobotState( vpRobot::STATE_VELOCITY_CONTROL);
  robot.getPosition( vpRobot::END_EFFECTOR_FRAME, ee_state);

  vpHomogeneousMatrix eedMee;
  vpHomogeneousMatrix eedMo(vpTranslationVector(0.200, -0.000, 0.000), vpQuaternionVector(-0.001, 0.707, 0.707, 0.001));
  vpHomogeneousMatrix wMo(vpTranslationVector(x_pos, y_pos, z_pos), vpQuaternionVector(x_or, y_or, z_or, w_or));
  vpHomogeneousMatrix l_zeroMee(vpTranslationVector( ee_state[0], ee_state[1], ee_state[2] ), vpThetaUVector(ee_state[3], ee_state[4], ee_state[5]));
  vpHomogeneousMatrix wMl_zero(vpTranslationVector(0.478,-0.312,0.080), vpQuaternionVector(-0.012, -0.001, 0.724, 0.690));
  //  - Translation: [0.478, -0.312, 0.080]
  // - Rotation: in Quaternion [-0.012, -0.001, 0.724, 0.690]

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
  task.setLambda( 0.5 );

  signal(SIGINT, my_function); 

  while(!stop_program){
    ros::Subscriber sub = n.subscribe("ARTBody", 1000, artCallback);
    // Update Matrixes
    vpColVector ee;
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

    // vpFeatureTranslation td( vpFeatureTranslation::cdMc );
    // vpFeatureThetaU tud( vpFeatureThetaU::cdRc );
    // task.addFeature( t, td );
    // task.addFeature( tu, tud );
    v_c = task.computeControlLaw();


    double maxVel = robot.getMaxTranslationVelocity();
    double maxRot = robot.getMaxRotationVelocity();

    float dt = 0.020;

    float nt_x = v_c[0]*dt;
    float nt_y = v_c[1]*dt;
    float nt_z = v_c[2]*dt;
    float nr_x = v_c[3]*dt;
    float nr_y = v_c[4]*dt;
    float nr_z = v_c[5]*dt;

    // calibration matrices 
    // starting point ee wrt world = 0.4514450757, 0.1934085473, 0.1953564438

    // std::cout << "Ee rotation: "    << wMee.getRotationMatrix() << std::endl;
    // std::cout << "Ee translation: " << wMee.getTranslationVector() << std::endl;

    float distance_x = 0.25;
    float distance_y = 0.25;
    float distance_z = 0.10;

    float x_forward  = 0.4514450757 + distance_x;
    float x_backward = 0.4514450757 - distance_x;
    float y_left     = 0.1934085473 + distance_y;
    float y_right    = 0.1934085473 - distance_y;
    float z_up       = 0.1953564438 + distance_z;
    float z_down     = 0.05;

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
    float rotx_min = -3.14;
    float rotx_max = 3.14;
    float roty_min = -0.9;
    float roty_max = 0.9;
    float rotz_min = -0.1;
    float rotz_max = 1.85;


    vpMatrix rotz_Mmin(3,3);
    //facing z axis, left , picture 1
    rotz_Mmin[0][0] = 0.744048;
    rotz_Mmin[0][1] = 0.668745;
    rotz_Mmin[0][2] = 0.00377827;
    rotz_Mmin[1][0] = 0.668563;
    rotz_Mmin[1][1] = -0.743957;
    rotz_Mmin[1][2] = 0.0198535;
    rotz_Mmin[2][0] = 0.016081;
    rotz_Mmin[2][1] = -0.0122408;
    rotz_Mmin[2][2] = -1.00022;

    vpMatrix rotz_Mmax(3,3);
    //facing z axis, right, picture 2
    rotz_Mmax[0][0] = -0.927193;
    rotz_Mmax[0][1] = 0.356925;
    rotz_Mmax[0][2] = -0.117302;
    rotz_Mmax[1][0] = 0.343372;
    rotz_Mmax[1][1] = 0.931804;
    rotz_Mmax[1][2] = 0.121157;
    rotz_Mmax[2][0] = 0.152482;
    rotz_Mmax[2][1] = 0.0720271;
    rotz_Mmax[2][2] = -0.986105;

    vpMatrix rotx_Mmin(3,3);
    //facing x axis, left , picture 3
    rotx_Mmin[0][0] = -0.0707442;
    rotx_Mmin[0][1] = 0.88029;
    rotx_Mmin[0][2] = -0.470029;
    rotx_Mmin[1][0] = 0.99237;
    rotx_Mmin[1][1] = 0.111671;
    rotx_Mmin[1][2] = 0.0597812;
    rotx_Mmin[2][0] = 0.105069;
    rotx_Mmin[2][1] = -0.462019;
    rotx_Mmin[2][2] = -0.881102;

    vpMatrix rotx_Mmax(3,3);
    //facing x axis, right, picture 4
    rotx_Mmax[0][0] = -0.195806;
    rotx_Mmax[0][1] = 0.83696;
    rotx_Mmax[0][2] = 0.51186;
    rotx_Mmax[1][0] = 0.980616;
    rotx_Mmax[1][1] = 0.151046;
    rotx_Mmax[1][2] = 0.128142;
    rotx_Mmax[2][0] = 0.0299226;
    rotx_Mmax[2][1] = 0.526807;
    rotx_Mmax[2][2] = -0.849954;

    vpMatrix roty_Mmin(3,3);
    //facing y axis, left, picture 5 
    roty_Mmin[0][0] = -0.0417367;
    roty_Mmin[0][1] = 0.993109;
    roty_Mmin[0][2] = -0.113288;
    roty_Mmin[1][0] = 0.866113;
    roty_Mmin[1][1] = 0.0925314;
    roty_Mmin[1][2] = 0.492066;
    roty_Mmin[2][0] = 0.498948;
    roty_Mmin[2][1] = -0.0775502;
    roty_Mmin[2][2] = -0.863643;

    vpMatrix roty_Mmax(3,3);
    //facing y axis, right, picture 6 
    roty_Mmax[0][0] = -0.0786967;
    roty_Mmax[0][1] = 0.997015;
    roty_Mmax[0][2] = 0.0247016;
    roty_Mmax[1][0] = 0.968078;
    roty_Mmax[1][1] = 0.082322;
    roty_Mmax[1][2] = -0.238517;
    roty_Mmax[2][0] = -0.239738;
    roty_Mmax[2][1] = 0.00514039;
    roty_Mmax[2][2] = -0.971258;

    vpMatrix rotationMax(3,3); 
    vpMatrix identityMatrix(3,3);
    identityMatrix.setIdentity(1.0);
    vpMatrix rotMatrixnext = wMee_next.getRotationMatrix();
    float threshold = 3.52e-8;
    
    // //Rotation Calibration
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
        x_backward<=next_world[0] && next_world[0]<=x_forward
        && y_right<=next_world[1] && next_world[1]<=y_left
        && z_down<=next_world[2] && next_world[2]<=z_up

        // && std::fabs((rotx_Mmin*rotMatrixnext.inverseByLU()-identityMatrix).det())>=threshold
        // && std::fabs((rotx_Mmax*rotMatrixnext.inverseByLU()-identityMatrix).det())>=threshold
        // && std::fabs((roty_Mmin*rotMatrixnext.inverseByLU()-identityMatrix).det())>=threshold
        // && std::fabs((roty_Mmax*rotMatrixnext.inverseByLU()-identityMatrix).det())>=threshold
        // && std::fabs((rotz_Mmin*rotMatrixnext.inverseByLU()-identityMatrix).det())>=threshold
        // && std::fabs((rotz_Mmax*rotMatrixnext.inverseByLU()-identityMatrix).det())>=threshold

        // && v_c[0]< maxVel
        // && v_c[1]< maxVel
        // && v_c[2]< maxVel
        // && v_c[3]< maxRot
        // && v_c[4]< maxRot
        // && v_c[5]< maxRot


        && !(isnan(v_c[0]))
        && !(isnan(v_c[1]))
        && !(isnan(v_c[2]))
        && !(isnan(v_c[3]))
        && !(isnan(v_c[4]))
        && !(isnan(v_c[5]))

        && isARTNormalized()
        ) 
    {
      // here set velocity
      v_c[0] = clip(v_c[0], -0.2 ,0.2);
      v_c[1] = clip(v_c[1], -0.2 ,0.2);
      v_c[2] = clip(v_c[2], -0.2 ,0.2);
      v_c[3] = clip(v_c[3], -0.2 ,0.2);
      v_c[4] = clip(v_c[4], -0.2 ,0.2);
      v_c[5] = clip(v_c[5], -0.2 ,0.2);

      // robot.setRobotState( vpRobot::STATE_VELOCITY_CONTROL);
      robot.setVelocity(vpRobot::END_EFFECTOR_FRAME, v_c);
      std::cout << "v_c" << v_c << std::endl;
    }
    else
    {
      std::cout << "Constraint violated " << std::endl;
      std::cout << "v_c" << v_c << std::endl;
      if (!isARTNormalized()){
        std::cout << "Art out of range "<< std::endl;
      }
      // // rotation matrix constraints
      // if (std::fabs((rotx_Mmin*rotMatrixnext.inverseByLU()-identityMatrix).det())<threshold){
      //   float res = std::fabs((rotx_Mmin*rotMatrixnext.inverseByLU()-identityMatrix).det());
      //   std::cout << "constraints rotx_Mmin "<< res << std::endl;
      // }
      // if (std::fabs((rotx_Mmax*rotMatrixnext.inverseByLU()-identityMatrix).det())<threshold){
      //   float res = std::fabs((rotx_Mmax*rotMatrixnext.inverseByLU()-identityMatrix).det());
      //   std::cout << "constraints rotx_Mmax "<< res << std::endl;
      // }

      // if (std::fabs((roty_Mmin*rotMatrixnext.inverseByLU()-identityMatrix).det())<threshold){
      //   float res = std::fabs((roty_Mmin*rotMatrixnext.inverseByLU()-identityMatrix).det());
      //   std::cout << "constraints roty_Mmin "<< res << std::endl;
      // }
      // if (std::fabs((roty_Mmax*rotMatrixnext.inverseByLU()-identityMatrix).det())<threshold){
      //   float res = std::fabs((roty_Mmax*rotMatrixnext.inverseByLU()-identityMatrix).det());
      //   std::cout << "constraints roty_Mmax "<< res << std::endl;
      // }

      // if (std::fabs((rotz_Mmin*rotMatrixnext.inverseByLU()-identityMatrix).det())<threshold){
      //   float res = std::fabs((rotz_Mmin*rotMatrixnext.inverseByLU()-identityMatrix).det());
      //   std::cout << "constraints rotz_Mmin "<< res << std::endl;
      // }
      // if (std::fabs((rotz_Mmax*rotMatrixnext.inverseByLU()-identityMatrix).det())<threshold){
      //   float res = std::fabs((rotz_Mmax*rotMatrixnext.inverseByLU()-identityMatrix).det());
      //   std::cout << "constraints rotz_Mmax "<< res << std::endl;
      // }

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
      // robot.setRobotState(vpRobot::STATE_STOP);
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
        robot.setRobotState(vpRobot::STATE_STOP);
        stop_program = true;
    }

    ros::spinOnce();
    loop_rate.sleep();
    

  } //while(1)

  } catch (const vpException &e) {
    std::cout << "ViSP exception: " << e.what() << std::endl;
    std::cout << "Stop the robot " << std::endl;
    robot.setRobotState(vpRobot::STATE_STOP);
    return EXIT_FAILURE;
  } catch (const franka::NetworkException &e) {
    std::cout << "Franka network exception: " << e.what() << std::endl;
    std::cout << "Check if you are connected to the Franka robot"
              << " or if you specified the right IP using --ip command line option set by default to 192.168.1.1. "
              << std::endl;
    return EXIT_FAILURE;
  } catch (const std::exception &e) {
    std::cout << "Franka exception: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  return 0;
}




