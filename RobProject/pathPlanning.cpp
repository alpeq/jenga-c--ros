#include <iostream>
#include <rw/rw.hpp>
#include <rw/trajectory/LinearInterpolator.hpp>
#include <rw/invkin/ClosedFormIKSolverUR.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/pathplanners/prm/PRMPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rwlibs/pathoptimization/pathlength/PathLengthOptimizer.hpp>
#include <vector>
#include <rw/pathplanning/QConstraint.hpp>
#include <rw/kinematics/Kinematics.hpp>

#include <rw/pathplanning/PlannerConstraint.hpp>

#include <rw/math.hpp>
#include <string>
#include <stdexcept>

using namespace std;
using namespace rw::invkin;
using namespace rw::common;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::pathplanning;
using namespace rw::proximity;
using namespace rw::trajectory;
using namespace rwlibs::pathplanners;
using namespace rwlibs::proximitystrategies;

#define MAXTIME 10.
#define DEBUG 0
#define OUT 1

#define LimitSphere 1.5  // REAL 40.25  //sqrt(20²+8²+34²)

#define Extend 0.05


/**
 * CollisionChecker Function
 **/
bool checkCollisions(Device::Ptr device, const State &state, const CollisionDetector &detector, const Q &q) {
    State testState;
    CollisionDetector::QueryResult data;
    bool colFrom;

    testState = state;
    device->setQ(q,testState);
    colFrom = detector.inCollision(testState,&data);
    if (colFrom) {
        cerr << "Configuration in collision: " << q << endl;
        cerr << "Colliding frames: " << endl;
        FramePairSet fps = data.collidingFrames;
        for (FramePairSet::iterator it = fps.begin(); it != fps.end(); it++) {
            cerr << (*it).first->getName() << " " << (*it).second->getName() << endl;
        }
        return false;
    }
    return true;
}

/**
 * getPoint: Get 3D point given robot, frame configuration and Q state
 **/
Transform3D<> getPoint(WorkCell::Ptr wc,Device::Ptr device, const State &state, const Q &q) {
    State testState;
    testState = state;
    device->setQ(q, testState);
    return Kinematics::worldTframe(wc->findFrame("PG70"), testState);
}

/**
 * CheckBorder: Check if given a Q value of the robot the x,y,z point is inside of the Static Zone
 **/
bool checkBorder(WorkCell::Ptr wc,Device::Ptr device, const State &state, const Q &q) {
    // Alternatively check .x() .y() .z()
    Transform3D<> point3D = getPoint(wc,device, state, q);
    Vector3D<> d = point3D.P();
    //cout << d.norm2() << endl;
    //if (d.norm2() >= LimitSphere)
    if (d.norm2() <= LimitSphere)
        return false;
    return true;
}

/**
 * Inverse Kinematics
 **/
Q inverseKinematics(const Device::Ptr device, const SerialDevice::Ptr sdevice, const State &state, Transform3D<double> TD){

    rw::invkin::ClosedFormIKSolverUR::Ptr urIK;
    urIK = new rw::invkin::ClosedFormIKSolverUR(sdevice, state);
    vector<Q> solutions = urIK->solve(TD, state);
    if (solutions.size() == 0) {
        cout << solutions.size() << endl;
        std::cout << "No solution found!"<< endl << endl << endl << endl << endl << endl << endl;
    }
    else
        if (DEBUG ==1)
            cout << "SOLUTION RETURNED!!!!!!!" << std::endl << std::endl << std::endl << std::endl << std::endl << std::endl << std::endl;
        return solutions[0];
}
  rw::trajectory::QPath linearInterpolatedPath(const rw::math::Q& start, const rw::math::Q& end,
                                               const double total_duration = 10.0, const double duration_step = 1.0)
  {
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



/**
   * PRM Planner for Static Zone, the roadmap is calculated outside of the planner
*/
QPath prmPath(PRMPlanner* prm, WorkCell::Ptr wc,Transform3D<> start, Transform3D<> end,Device::Ptr device,SerialDevice::Ptr sdevice,State state,  QMetric::Ptr metric){

    QPath path;
    // Constraints
    CollisionDetector detector(wc, ProximityStrategyFactory::makeDefaultCollisionStrategy());

    // Calculate Inverse
    Q from = inverseKinematics(device, sdevice, state, start);
    Q to = inverseKinematics(device, sdevice, state, end);
    // First Checking
    if (!checkCollisions(device, state, detector, from))
        return 0;
    if (!checkCollisions(device, state, detector, to))
        return 0;

    /*** Evaluation and Path calculation ***/
    cout << "Calculate path from " << from << " to " << to << endl;

    //Call planner
    prm->query(from,to,path,MAXTIME);

    return path;
  }

/**
 * RRT Planner for Dynamic Zone
 */
QPath rrtPath(WorkCell::Ptr wc,Transform3D<> start, Transform3D<> end,Device::Ptr device,SerialDevice::Ptr sdevice,State state,  QMetric::Ptr metric){

  QPath path;
  // Constraints
  CollisionDetector detector(wc, ProximityStrategyFactory::makeDefaultCollisionStrategy());
  PlannerConstraint constraint = PlannerConstraint::make(&detector,device,state);
  QSampler::Ptr sampler = QSampler::makeConstrained(QSampler::makeUniform(device),constraint.getQConstraintPtr());
  QToQPlanner::Ptr rrt = RRTPlanner::makeQToQPlanner(constraint, sampler, metric, Extend, RRTPlanner::RRTConnect);

  // Calculate Inverse
  Q from = inverseKinematics(device, sdevice, state, start);
  Q to = inverseKinematics(device, sdevice, state, end);
  // First Checking
  if (!checkCollisions(device, state, detector, from))
      return 0;
  if (!checkCollisions(device, state, detector, to))
      return 0;

  /*** Evaluation and Path calculation ***/
  cout << "Calculate path from " << from << " to " << to << endl;

  rrt->query(from,to,path,MAXTIME); //run planner

  return path;
}

QPath pathPlanner(PRMPlanner* prm, WorkCell::Ptr wc,Transform3D<> FROM, Transform3D<> TO,Device::Ptr device,SerialDevice::Ptr sdevice,State state,  QMetric::Ptr metric){

    int i = 0;
    Q lastCorrect;
    QPath pathPRM, pathRRT,path;

    pathPRM = prmPath(prm,wc,FROM,TO,device,sdevice,state, metric);
    /*** Output Path ***/
    for (QPath::iterator it = pathPRM.begin(); it < pathPRM.end(); it++) {
        //cout << *it << endl;
        // Check if the points of the path are outside of the Static Zone
        if (checkBorder(wc,device, state, *it) == 0){
            if (it == pathPRM.begin())
                lastCorrect = *(pathPRM.begin());
            else
                lastCorrect = *(--it);
            // Get the point of the new start of the dynamic zone
            Transform3D<> newStart = getPoint(wc,device, state, lastCorrect);
                    //NEED TO CHANGE FROM FOR lastCorrect  (and adjust the sphere to worldFrame Values ASK SAlman for How to obtain from now)
            pathRRT = rrtPath(wc,FROM,TO,device,sdevice,state,metric);
            // Merge the paths
            path = QPath(pathPRM.begin(),pathPRM.begin()+i);
            copy(pathRRT.begin(), pathRRT.end(), std::inserter(path, path.end()));
            break;
        }
        i++;
    }
  return path;
}


int main() {

    /*** Load Cell and device ***/
    const string wcFile = "/home/ali/WorkSpace/RobWork/WorkCells/WorkStation_1/WC1_Scene.wc.xml";
    const string deviceName = "UR1";
    int i = 0;

    WorkCell::Ptr wc = WorkCellLoader::Factory::load(wcFile);
    State state = wc->getDefaultState();
    Device::Ptr device = wc->findDevice(deviceName);
    SerialDevice::Ptr sdevice = wc->findDevice<SerialDevice>(deviceName);
    cout << "Trying to use workcell " << wcFile << " and device " << deviceName << endl;
    if (device == NULL) {
        cerr << "Device: " << deviceName << " not found!" << endl;
        return 0;
    }
    if (sdevice == NULL) {      // Some methods require serial devices
        cerr << "SerialDevice: " << deviceName << " not found!" << endl;
        return 0;
    }

    /***  Get Path  ***/
    // Constraints
    Timer local,total;
    QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();
    CollisionDetector detector(wc, ProximityStrategyFactory::makeDefaultCollisionStrategy());
    PlannerConstraint constraint = PlannerConstraint::make(&detector,device,state);

    // Build the road map just once
    PRMPlanner* prm = new PRMPlanner(device.get(), state, &detector, Extend); //input device as (rw::models::Device*)
    prm->setCollisionCheckingStrategy(PRMPlanner::LAZY);
    prm->setNeighSearchStrategy(PRMPlanner::BRUTE_FORCE);
    prm->setShortestPathSearchStrategy(PRMPlanner::A_STAR);
    prm->buildRoadmap(5000);

    /*** Change the start/end values and create different queries ***/
    while (i < 3){
        i++;
        local.resetAndResume();
        // Point Selection //
        Vector3D<> startP(0.52, -0.05, 0.84);
        RPY<> startR(-1.6, -0.1, -1.7);
        //Vector3D<> V1(20, -34, 5); //RPY<> R1(-3, 0, 2); Values obtain from world-PG17 RWS
        Vector3D<> endP(0.24, -0.85, -0.15);
        RPY<> endR(-2.8, 1.4, -2.8);

        // Transformation Matrix
        Transform3D<> FROM(startP, startR.toRotation3D());
        Transform3D<> TO(endP, endR.toRotation3D());

        // Call to Planer
        QPath path = pathPlanner(prm, wc,FROM,TO,device,sdevice,state, metric);

        /** Path Optimization **/
        rwlibs::pathoptimization::PathLengthOptimizer PathLengthOptimizer(constraint, metric); // Optimize Pathpranning
        QPath pathOpti= PathLengthOptimizer.pathPruning(path); // Optimize Path using PathPruning method
        //QPath pathOpti= PathLengthOptimizer.shortCut(path); // Optimize Path using shortcut method

        local.pause();

        /*** Output Path ***/
        if (OUT == 1){
            cout << "Path of length " << path.size() << " found in " << local.getTime() << " seconds." << endl;
            cout << "Optimized Path of length " << pathOpti.size() << " found "<< endl;
            if (local.getTime() >= MAXTIME) {
                cout << "Notice: max time of " << MAXTIME << " seconds reached." << endl;
            }
            // Original Path
            for (QPath::iterator it = path.begin(); it < path.end(); it++) {
                cout << *it << endl;
            }

            cout << "..." << endl<< endl;
            // Optimized Path
            cout << "Optimized Path" << endl;
            for (QPath::iterator it = pathOpti.begin(); it < pathOpti.end(); it++) {
                cout << *it << endl;
            }
        }
    }
    local.pause();
    cout << " Total time: ." << total.getTime() << " seconds." << endl;
    return 0;

}
