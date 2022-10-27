#include <iostream>

#include <visp3/robot/vpRobotFranka.h>
#include <visp3/vs/vpServo.h>
#include <visp_ros/vpROSGrabber.h>
#include <art_publisher/marker.h>
#include <art_publisher/body.h>
#include <ros/ros.h>
#include <visp3/visual_features/vpFeatureThetaU.h>
#include <visp3/visual_features/vpFeatureTranslation.h>

art_publisher::body local_pos;
int poseinX;
bool opt_adaptive_gain = true;
vpColVector v_c( 6);

void artCallback (const art_publisher::body::ConstPtr &msg)
{
    ROS_INFO("%f", msg->bodies[0].pose.position.x);
    ROS_INFO("%s", typeid(msg->bodies[0].pose.position.x).name());
    poseinX = msg->bodies[0].pose.position.x;

}

int main(int argc, char **argv)
{
  std::string opt_robot_ip = "172.16.0.2";
  std::string opt_position_filename = "position.pos";
  vpRobotFranka robot;
  v_c = 0;

  try {

  ros::init(argc, argv, "control_node");

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("ARTBody", 1000, artCallback);

  robot.connect("172.16.0.2");
  std::cout << "connected " << std::endl;

  vpServo task;
  vpColVector joint_state;
  vpColVector ee_state;
  vpColVector camera_state;
  vpColVector reference_frame;
  
  robot.setRobotState( vpRobot::STATE_VELOCITY_CONTROL );
  task.setServo( vpServo::EYEINHAND_CAMERA);
  task.setInteractionMatrixType( vpServo::CURRENT);
  
  robot.getPosition( vpRobot::END_EFFECTOR_FRAME, ee_state );

  // Matrix initialization
  vpHomogeneousMatrix eedMo( vpTranslationVector( 0, 0.0, 0.3), vpQuaternionVector(0.7071068, 0.7071068, 0, 0 )) ;
  vpHomogeneousMatrix fMc( vpTranslationVector(0.575, 0.000000046, 0.685) , vpQuaternionVector(-0.5, 0.5, -0.5, 0.5));
  vpHomogeneousMatrix fMee( vpTranslationVector( ee_state[0], ee_state[1], ee_state[2] ), vpThetaUVector(ee_state[3], ee_state[4], ee_state[5]) );
  vpHomogeneousMatrix cMf, cMee, eedMee, cMo, oMo;

  // Matrix multiplication
  cMf = fMc.inverse();
  cMee = cMf * fMee;
  eedMee = eedMo * cMo.inverse() * cMee;

  // Features
  vpFeatureTranslation t( vpFeatureTranslation::cdMc );
  vpFeatureThetaU tu( vpFeatureThetaU::cdRc );
  t.buildFrom( eedMee );
  tu.buildFrom( eedMee );
  vpFeatureTranslation td(  vpFeatureTranslation::cdMc );
  vpFeatureThetaU tud( vpFeatureThetaU::cdRc );
  task.addFeature( t, td );
  task.addFeature( tu, tud );

  if ( opt_adaptive_gain )
  {
    vpAdaptiveGain lambda( 4, 1.2, 25 ); // lambda(0)=4, lambda(oo)=1.2 and lambda'(0)=25
    task.setLambda( lambda );
  }
  else
  {
    task.setLambda( 2.5 );
  }

  while(1){

    // Updated Matrixes
    vpColVector ee;
    robot.getPosition( vpRobot::END_EFFECTOR_FRAME, ee );
    vpHomogeneousMatrix ufMee( vpTranslationVector( ee[0], ee[1], ee[2] ), vpThetaUVector(ee[3], ee[4], ee[5]) );

    cMf = fMc.inverse();
    cMee = cMf*ufMee;
    task.set_cVf(cMf);

    // Update visual features 
    vpHomogeneousMatrix oMc;
    oMc = cMo.inverse();

    eedMee = eedMo * oMc * cMee;
    t.buildFrom( eedMee );
    tu.buildFrom( eedMee);

    v_c = task.computeControlLaw();
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

    vpMatrix rotationMax(3,3); 
    vpMatrix identityMatrix(3,3);
    identityMatrix.setIdentity(1.0);
    vpMatrix rotMatrixnext = fMee_next.getRotationMatrix();
    float threshold = 3.52e-8;
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
        ) 
    {

      //here set velocity

    }
    else
    {
      std::cout << "Constraint violated " << std::endl;

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
      // here set velocity
    }

    std::cout << "End effector state" << ee.t()  << std::endl;
    std::cout << "ART pose in X: " << poseinX  << std::endl;

  }

  ros::spin();

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
