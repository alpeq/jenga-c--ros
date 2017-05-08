#include <caros/test/universal_robot_test.h>

#include <ros/ros.h>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "simple_caros_universalrobot_demo_using_move_servo_q");

//  const rw::math::Q q_desire(6, 0, -1.37, -0.77, -1.37, 0, 0);
  const rw::math::Q q_target1(6, 0, -1.97, -0.77, -1.97, 0, 0);



  rw::math::Vector3D<> V0(0.042, -0.107, 0.885);
  rw::math::RPY<> R0(-3.082, 0.025, -1.571);
  rw::math::Transform3D<> TD(V0, R0.toRotation3D()); // CREATE DESIRED TRANSFORMATION MATRIX




  UrTest ur_test;
  bool ret = false;
  rw::math::Q q_target2 = ur_test.inverseKinematics(TD);

//  std::vector<rw::math::Q> q_desire = ur_test.inverseK(TD);
  ret = ur_test.testMovePtp(q_target1,q_target2);

  if (!ret)
  {
    ROS_ERROR_STREAM("Could not properly do the testMoveServoQ");
    return 1;
  }

  return 0;
}
