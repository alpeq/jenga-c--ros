
#include <caros/serial_device_si_proxy.h>
#include <caros/common_robwork.h>
#include <stdexcept>

#include <caros/test/universal_robot_test.h>
#include <ros/ros.h>
#include <rw/pathplanning/PlannerConstraint.hpp>
#include <rw/pathplanning/QToQPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rw/math/MetricFactory.hpp>
#include <rw/pathplanning/QSampler.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyPQP.hpp>
#include <rw/trajectory/LinearInterpolator.hpp>
#include <iostream>
#include <vector>
#include <rw/math.hpp>
#include <string>
#include <rw/rw.hpp>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "simple_caros_universalrobot_demo_using_move_ptp");
//  const rw::math::Q q_target2(6, 1.643, -0.891, 1.023, -1.573, 0.000, 0.000);
  const rw::math::Q q_target1(6, 0, -1.67, 0, -1.67, 0, 0);

  UrTest ur_test;
  bool ret = false;

//  rw::math::Vector3D<> V0(-0.001, -0.191, 0.998);
//  rw::math::RPY<> R0(-3.142, 0.002, -1.571);  // RETURNS Q

//  rw::math::Vector3D<> V0(0.04411900, -0.1064460, 0.8854390);
//  rw::math::RPY<> R0(-3.07893533, 0.027523842, -1.5707963268);
// 0.01m = 10mm

//  for (float z = 1.0; z > 0.405; --z*0.03) {
      rw::math::Vector3D<> V0(-0.002, 0.092, 0.777);
      rw::math::RPY<> R0(-3.120, 0.002, -1.571);
      rw::math::Transform3D<> TD(V0, R0.toRotation3D()); // CREATE DESIRED TRANSFORMATION MATRIX

    //rw::math::Vector3D<> V0(-0.002, -0.192, 0.99-);


  std::cout << "INPUT TRANSFORMATION MATRIX" << TD << std::endl << std::endl;

//    rw::math::Q q_target2 = ur_test.getCurrentJointConfiguration();
    rw::math::Q q_target2 = ur_test.inverseKinematics(TD);
    std::cout << "INVKIN TRANSFORM joint config " << q_target2 << std::endl << std::endl;



    ret = ur_test.testMovePtp(q_target1,q_target2);

    if (!ret)
    {
      ROS_ERROR_STREAM("Could not properly do the testMovePtp");
      return 1;
    }

  return 0;
}


