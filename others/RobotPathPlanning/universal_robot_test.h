#ifndef CAROS_TEST_UNIVERSAL_ROBOT_TEST_H
#define CAROS_TEST_UNIVERSAL_ROBOT_TEST_H

#include <caros/serial_device_si_proxy.h>
#include <caros/common_robwork.h>

#include <rw/pathplanning/PlannerConstraint.hpp>
#include <rw/pathplanning/QToQPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/pathplanners/prm/PRMPlanner.hpp>
#include <rw/math/MetricFactory.hpp>
#include <rw/pathplanning/QSampler.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyPQP.hpp>
#include <rw/trajectory/LinearInterpolator.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <ros/ros.h>
#include <string>
#include <stdexcept>
#include <rw/invkin/ClosedFormIK.hpp>
#include <rw/invkin/JacobianIKSolver.hpp>
#include <iostream>
#include <vector>
#include <rw/rw.hpp>
#include <rw/math.hpp>
#include <rw/kinematics/Kinematics.hpp>

/* WARNING: USE AT YOUR OWN RISK!
 * None of this is code is guaranteed to not make the robot crash into anything or do unexpected behaviour!
 * It is highly recommended that you understand what the code does before you run it!
 *
 * To minimise risk, please use (very) small q_change values!
 */

/* This class is supposed to make it a bit easier to reuse the test code.
 * To use another planner or collision detector, simply add a member function that the user can invoke and override the
 * defaults (before invoking the test functions)
 */


class UrTest
{
    public:
        UrTest() : nodehandle_("~"), sdsip_(nodehandle_, "ur_simple_demo_node")
        {
            initWorkCell();
            initDevice();
            initPathPlannerWithCollisionDetector();
        }

        virtual ~UrTest()
        {
        /* Empty */
        }

        rw::math::Q getCurrentJointConfiguration()
      {
          /* Make sure to get and operate on fresh data from the serial device
           * It's assumed that the serial device is not moving
           * ^- That could be asserted / verified using sdsip.isMoving()
           * However other sources could invoke services on the UR that causes it to move...
           */
          ros::Time current_timestamp = ros::Time::now();
          ros::Time obtained_timestamp = sdsip_.getTimeStamp();
          while (current_timestamp > obtained_timestamp)
          {
            ros::Duration(0.1).sleep();  // In seconds
            ros::spinOnce();
            obtained_timestamp = sdsip_.getTimeStamp();
          }

          return sdsip_.getQ();
        }


        rw::math::Q inverseKinematics(rw::math::Transform3D<double> TD) {
            // Default state is Obtained from Workcell
            rw::kinematics::State state = workcell_->getDefaultState();

            // calculate base to end transform
            rw::math::Transform3D<> bTe = device_->baseTend(state);
            //std::cout << "baseTend found!"<< bTe << std::endl << std::endl << std::endl << std::endl << std::endl << std::endl << std::endl;
            rw::invkin::JacobianIKSolver IK_solver(device_, state);
            IK_solver.setCheckJointLimits(true);
            IK_solver.setEnableInterpolation(true);
            std::vector<rw::math::Q> solutions = IK_solver.solve(TD, state);
            if (solutions.size() == 0) {
                std::cout << "No solution found!"<< std::endl << std::endl << std::endl << std::endl << std::endl << std::endl << std::endl;
            }
            else
                std::cout << "SOLUTION RETURNED!!!!!!!" << std::endl << std::endl << std::endl << std::endl << std::endl << std::endl << std::endl;
                return solutions[0];
        }

        //rw::math::Q q_target2 = inverseKinematics(device_, state, TD);


        //rw::math::Q current_configuration = getCurrentJointConfiguration();

        bool testMovePtp(const rw::math::Q q_target1, const rw::math::Q q_target2)
        {
            if (!doTestMovePtp(q_target1, q_target2))
            {
                std::cout<<"Check 3"<<std::endl;
                return false;
            }
        ROS_INFO_STREAM("Waiting a little moment for the movement to be finished");
        ros::Duration(5).sleep();  // In seconds

        if (!doTestMovePtp(-q_target1, -q_target2))
        {
            std::cout<<"Check 4"<<std::endl;
            return false;
        }
        ROS_INFO_STREAM("Waiting a little moment for the movement to be finished");
        ros::Duration(5).sleep();  // In seconds

        return true;
        }

        bool testMoveServoQ(const rw::math::Q q_target1, const rw::math::Q q_target2)
        {
            if (!doTestMoveServoQ(q_target1, q_target2))
            {
                return false;
            }
        ROS_INFO_STREAM("Waiting a little moment for the movement to be finished");
        ros::Duration(5).sleep();  // In seconds

        if (!doTestMoveServoQ(-q_target1, -q_target2))
        {
            return false;
        }
        ROS_INFO_STREAM("Waiting a little moment for the movement to be finished");
        ros::Duration(5).sleep();  // In seconds
        return true;
        }

  //---------------------------------------------------------


    protected:
    void initWorkCell()
    {
        workcell_ = caros::getWorkCell(); // Workcell Obtained
        if (workcell_ == NULL)
        {
            ROS_ERROR("No workcell was loaded - exiting...");
            throw std::runtime_error("Not able to obtain a workcell.");
        }
    }

    void initDevice()
    {
        std::string device_name;
        if (!nodehandle_.getParam("device_name", device_name))
        {
        ROS_FATAL_STREAM("The parameter '" << nodehandle_.getNamespace()
                                         << "/device_name' was not present on the parameter "
                                            "server! This parameter has to be specified "
                                            "for this test-node to work properly.");
        throw std::runtime_error("Not able to obtain device name.");
        }

        ROS_DEBUG_STREAM("Looking for the device '" << device_name << "' in the workcell.");
        device_ = workcell_->findDevice(device_name); // Device Obtained from Workcell
        if (device_ == NULL)
        {
        ROS_FATAL_STREAM("Unable to find device " << device_name << " in the loaded workcell");
        throw std::runtime_error("Not able to find the device within the workcell.");
        }
    }


    void initPathPlannerWithCollisionDetector()
    {
        //Default state is Obtained from Workcell
        rw::kinematics::State state = workcell_->getDefaultState();

    /* Collision detector */
        auto detector = rw::common::ownedPtr(
        new rw::proximity::CollisionDetector(workcell_, rwlibs::proximitystrategies::ProximityStrategyPQP::make()));

    /* PlannerConstraint that uses the collision detector to verify that the _start_ and _end_ configurations are
     * collision free and that the edge(s) between those is/are also collision free. */
        const rw::pathplanning::PlannerConstraint planner_constraint =
        rw::pathplanning::PlannerConstraint::make(detector, device_, state);

    /* Just using a really simple path planner (straight line in the configuration space) */
        //planner_ = rw::pathplanning::QToQPlanner::make(planner_constraint);
        //planner_ = rwlibs::pathplanners::RRTQToQPlanner::makeConnect(planner_constraint, rw::pathplanning::QSampler::makeUniform(device_),rw::math::MetricFactory::makeEuclidean<rw::math::Q>(), 0.1);
        planner_ = rwlibs::pathplanners::RRTQToQPlanner::makeBidirectional(planner_constraint, rw::pathplanning::QSampler::makeUniform(device_),rw::math::MetricFactory::makeEuclidean<rw::math::Q>(), 0.1);


    //PRM PLANNER TRY // FIRST WE HAVE TO BUILD A ROAD MAP

   // planner_ = rwlibs::pathplanners::PRMPlanner(planner_constraint, rw::pathplanning::QSampler::makeUniform(device_), 0.1, device_, state);
      //Device = device_
      //State = state
      //Constraint = planner_constraint
      //Sampler = rw::pathplanning::QSampler::makeUniform(device_)
      //Metric = rw::math::MetricFactory::makeEuclidean<rw::math::Q>()
      //Extend = 0.1

  }


  // Current State to Target 1------------------------------------
    rw::trajectory::QPath getQPath(const rw::math::Q start_conf , const rw::math::Q target_1)
    {
        rw::math::Q start_configuration = start_conf;
        //rw::math::Q end_configuration = start_configuration + rw::math::Q(start_configuration.size(), q_change);

        rw::math::Q end_configuration = target_1; // Assign Target1 Configuration for Robot

        rw::trajectory::QPath path1;
        bool valid_path1 = false;
        ROS_ASSERT(planner_);
        valid_path1 = planner_->query(start_configuration, end_configuration, path1);
        if (!valid_path1)
        {
            ROS_ERROR_STREAM("Could not find a path from '" << start_configuration << "' to '" << end_configuration << "'.");
            throw std::runtime_error("No valid path found.");
        }

        return path1;
    }

  /*
  // Current State to Target 2------------------------------------
    rw::trajectory::QPath getQPath2(const rw::math::Q q_target1, const rw::math::Q q_target2)
    {
        rw::math::Q start_configuration = q_target1;
        //rw::math::Q end_configuration = start_configuration + rw::math::Q(start_configuration.size(), q_change);

        rw::math::Q end_configuration = q_target2; // Assign Target Configuration for Robot

        rw::trajectory::QPath path2;
        bool valid_path2 = false;
        ROS_ASSERT(planner_);
        valid_path2 = planner_->query(start_configuration, end_configuration, path2);
        if (!valid_path2)
        {
            ROS_ERROR_STREAM("Could not find a path from '" << start_configuration << "' to '" << end_configuration << "'.");
            throw std::runtime_error("No valid path found.");
        }

        return path2;
    }
  */
//-------------------------------------------------------------------

  // LinearInterpolatedPath Function Defined
    rw::trajectory::QPath linearInterpolatedPath(const rw::math::Q& start, const rw::math::Q& end,
                                               const double total_duration = 10.0, const double duration_step = 1.0)
    {
        ROS_ASSERT(duration_step > 0);
        ROS_ASSERT(duration_step < total_duration);
        rw::trajectory::QLinearInterpolator interpolator(start, end, total_duration);
        rw::trajectory::QPath path;
        path.push_back(start);
        for (double t = duration_step; t <= (total_duration - duration_step); t += duration_step)
        {
            path.push_back(interpolator.x(t));
        }
        path.push_back(end);
        return path;
    }



    bool doTestMovePtp(const rw::math::Q q_target1, const rw::math::Q q_target2)
    {
        bool return_status = true;
        std::cout<<"Check 5"<<std::endl;
        rw::math::Q current = getCurrentJointConfiguration();
        rw::trajectory::QPath path1 = getQPath(current, q_target1);
        rw::trajectory::QPath path2 = getQPath(q_target1, q_target2);
        std::cout<<"Check 6"<<std::endl;
        for (const rw::math::Q& p1 : path1) // Execute Path1
        {
            ROS_INFO_STREAM("Ask to movePtp to '" << p1 << "'.");
            bool ret = false;
            std::cout<<"Check 7"<<std::endl;
            ret = sdsip_.movePtp(p1);
            std::cout<<"Check 8"<<std::endl;
            if (!ret)
            {
                std::cout<<"Check 9"<<std::endl;
                return_status = false;
                ROS_ERROR_STREAM("The serial device didn't acknowledge the movePtp command.");
            }
        }

        ros::Duration(5).sleep();  // Delay After first Target Joint Configuration

        for (const rw::math::Q& p2 : path2) // Execute Path2
        {
            ROS_INFO_STREAM("Ask to movePtp to '" << p2 << "'.");
            bool ret = false;
            std::cout<<"Check 7"<<std::endl;
            ret = sdsip_.movePtp(p2);
            std::cout<<"Check 8"<<std::endl;
            if (!ret)
            {
                std::cout<<"Check 9"<<std::endl;
                return_status = false;
                ROS_ERROR_STREAM("The serial device didn't acknowledge the movePtp command.");
            }
        }

        return return_status;
    }

    bool doTestMoveServoQ(const rw::math::Q q_target1, const rw::math::Q q_target2)
    {
        bool return_status = true;
        rw::math::Q current = getCurrentJointConfiguration();
        rw::trajectory::QPath path1 = getQPath(current,q_target1);
        rw::trajectory::QPath path2 = getQPath(q_target1, q_target2);

        ROS_ASSERT(path1.size() == 2);
        rw::math::Q start_configuration = path1.at(0);
        rw::math::Q end_configuration1 = path1.at(1);

        ROS_ASSERT(path2.size() == 2);
        end_configuration1 = path2.at(0);
        rw::math::Q end_configuration2 = path2.at(1);

        //replace the path with an interpolated path
        path1 = linearInterpolatedPath(start_configuration, end_configuration1);
        path2 = linearInterpolatedPath(end_configuration1, end_configuration2);

        for (const rw::math::Q& p1 : path1)
        {
            ROS_INFO_STREAM("Ask to moveServoQ to '" << p1 << "'.");
            bool ret = false;
            ret = sdsip_.moveServoQ(p1);
            if (!ret)
            {
                return_status = false;
                ROS_ERROR_STREAM("The serial device didn't acknowledge the moveServoQ command.");
            }
        }

        ros::Duration(5).sleep();  //Delay After first Target Joint Configuration

        for (const rw::math::Q& p2 : path2)
        {
            ROS_INFO_STREAM("Ask to moveServoQ to '" << p2 << "'.");
            bool ret = false;
            ret = sdsip_.moveServoQ(p2);
            if (!ret)
            {
                return_status = false;
                ROS_ERROR_STREAM("The serial device didn't acknowledge the moveServoQ command.");
            }
        }
        return return_status;
    }

    protected:
    ros::NodeHandle nodehandle_;
    caros::SerialDeviceSIProxy sdsip_;

    rw::models::WorkCell::Ptr workcell_;
    rw::models::Device::Ptr device_;
    rw::pathplanning::QToQPlanner::Ptr planner_;
    //rwlibs::pathplanners::prm::PRMPlanner::Ptr planner_;
};

#endif  // CAROS_TEST_UNIVERSAL_ROBOT_TEST_H
