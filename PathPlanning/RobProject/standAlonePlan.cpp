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

#define MAXTIME 30.

#define OUT 11

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
 * Distance:
 **/
double distance(Transform3D<> pT1, Transform3D<> pT2) {

    Vector3D<> p1 = pT1.P();
    Vector3D<> p2 = pT2.P();
    Vector3D<> d = Vector3D<>(dot(p2.x(),p2)-dot(p1.x(),p1),dot(p2.y(),p2)-dot(p1.y(),p1),dot(p2.z(),p2)-dot(p1.z(),p1));
    return d.norm2();
}

double getTdistance(rw::trajectory::QPath path,WorkCell::Ptr wc,Device::Ptr device, const State &state) {

    double total = 0;
    for (QPath::iterator it = path.begin(); it < (path.end()-1); it++) {
        Q first = *(it);
        Q second =*(it+1) ;

        total += distance(getPoint(wc,device, state, first) , getPoint(wc,device, state, second) );

    }
    return total;
}


//Collision CHeck Function defined
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

//Inverse Kinematics function defined
Q inverseKinematics(const Device::Ptr device, const SerialDevice::Ptr sdevice, const State &state, Transform3D<double> TD){

    rw::invkin::ClosedFormIKSolverUR::Ptr urIK;
    urIK = new rw::invkin::ClosedFormIKSolverUR(sdevice, state);
    //urIK.setCheckJointLimits(true);
    vector<Q> solutions = urIK->solve(TD, state);
    if (OUT != 11)
        cout << solutions.size() << endl;
    if (solutions.size() == 0) {
        std::cout << "No solution found!"<< endl << endl << endl << endl << endl << endl << endl;
    }
    return solutions[1];
}

// LinearInterpolatedPath Function Defined
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


int main() {
    const string wcFile = "/home/ali/WorkSpace/Rovi/WorkCell_scenes/WorkStation_Medium/WC1_Scene.wc.xml";
    const string deviceName = "UR1";
    cout << "Trying to use workcell " << wcFile << " and device " << deviceName << endl;

    WorkCell::Ptr wc = WorkCellLoader::Factory::load(wcFile);

    Device::Ptr device = wc->findDevice(deviceName);
    if (device == NULL) {
        cerr << "Device: " << deviceName << " not found!" << endl;
        return 0;
    }

    const SerialDevice::Ptr sdevice = wc->findDevice<SerialDevice>(deviceName);
    if (sdevice == NULL) {
        cerr << "SerialDevice: " << deviceName << " not found!" << endl;
        return 0;
    }

    const State state = wc->getDefaultState();
    CollisionDetector detector(wc, ProximityStrategyFactory::makeDefaultCollisionStrategy());
    PlannerConstraint constraint = PlannerConstraint::make(&detector,device,state);

    //------------------------------------------------------------------------------------------
    //        //Points for Easy WorkCell

//          /** Points of our Planning **/
//          Vector3D<> sp1(-0.18, -0.06, 0.82); RPY<> sr1(-1.3, -1.5, 1.5);
//          Vector3D<> ep1(0.0, -0.82, 0.28); RPY<> er1(-1.4, 0.0, -3.0);
//          // Transformation Matrix
//          Transform3D<> F1(sp1, sr1.toRotation3D()); Transform3D<> T1(ep1, er1.toRotation3D());

//          Vector3D<> sp2(-0.18, -0.06, 0.82); RPY<> sr2(-1.3, -1.5, 1.5);
//          Vector3D<> ep2(0.18, -0.82, 0.28); RPY<> er2(-1.4, 0.0, -3.0);
//          // Transformation Matrix
//          Transform3D<> F2(sp2, sr2.toRotation3D()); Transform3D<> T2(ep2, er2.toRotation3D());

//          Vector3D<> sp3(-0.18, -0.06, 0.82); RPY<> sr3(-1.3, -1.5, 1.5);
//          Vector3D<> ep3(0.18, -0.82, 0.28); RPY<> er3(-1.4, 0.0, -3.0);
//          // Transformation Matrix
//          Transform3D<> F3(sp3, sr3.toRotation3D()); Transform3D<> T3(ep3, er3.toRotation3D());

//          Vector3D<> sp4(-0.18, -0.06, 0.82); RPY<> sr4(-1.3, -1.5, 1.5);
//          Vector3D<> ep4(0.049, -0.92, 0.178); RPY<> er4(-1.3, -0.3, -3.1);
//          // Transformation Matrix
//          Transform3D<> F4(sp4, sr4.toRotation3D()); Transform3D<> T4(ep4, er4.toRotation3D());

//          Vector3D<> sp5(-0.18, -0.06, 0.82); RPY<> sr5(-1.3, -1.5, 1.5);
//          Vector3D<> ep5(0.30, -0.80, 0.28); RPY<> er5(-0.9, 0.0, 2.5);
//          // Transformation Matrix
//          Transform3D<> F5(sp5, sr5.toRotation3D()); Transform3D<> T5(ep5, er5.toRotation3D());


      //Points for Medium WorkCell
      /** Points of our Planning **/
      //Vector3D<> sp1(0.47, -0.65, 0.28); RPY<> sr1(-0.7, 0.1, 2.8);
      Vector3D<> sp1(0.35, -0.65, 0.28); RPY<> sr1(-0.7, 0.1, 2.8);
      Vector3D<> ep1(-0.38, -0.69, 0.28); RPY<> er1(-0.5, 1.3, 1.2);
      // Transformation Matrix
      Transform3D<> F1(sp1, sr1.toRotation3D()); Transform3D<> T1(ep1, er1.toRotation3D());

      Vector3D<> sp2(0.35, -0.65, 0.28); RPY<> sr2(-0.7, 0.1, 2.8);
      Vector3D<> ep2(-0.3, -0.74, 0.28); RPY<> er2(-0.4, 1.3, 1.2);
      // Transformation Matrix
      Transform3D<> F2(sp2, sr2.toRotation3D()); Transform3D<> T2(ep2, er2.toRotation3D());

      //Vector3D<> sp3(0.35, -0.75, 0.28); RPY<> sr3(-0.7, 0.1, 2.8);
      Vector3D<> sp3(0.35, -0.65, 0.28); RPY<> sr3(-0.7, 0.1, 2.8);
      Vector3D<> ep3(-0.45, -0.61, 0.34); RPY<> er3(1.6, 0.4, 3.0);
      // Transformation Matrix
      Transform3D<> F3(sp3, sr3.toRotation3D()); Transform3D<> T3(ep3, er3.toRotation3D());

      //Vector3D<> sp4(0.35, -0.63, 0.28); RPY<> sr4(-0.7, 0.1, 2.8);
      Vector3D<> sp4(0.35, -0.65, 0.28); RPY<> sr4(-0.7, 0.1, 2.8);
      //Vector3D<> ep4(-0.47, -0.66, 0.47); RPY<> er4(-2.3, -1.4, -1.8);
      Vector3D<> ep4(-0.51, -0.45, 0.29); RPY<> er4(0.7, 0.5, 2.7);
      // Transformation Matrix
      Transform3D<> F4(sp4, sr4.toRotation3D()); Transform3D<> T4(ep4, er4.toRotation3D());

//        Vector3D<> sp5(0.42, -0.74, 0.28); RPY<> sr5(-0.6, 0.1, 2.8);
      Vector3D<> sp5(0.35, -0.65, 0.28); RPY<> sr5(-0.7, 0.1, 2.8);
      Vector3D<> ep5(-0.36, -0.61, 0.18); RPY<> er5(1.0, 0.4, 2.7);
      // Transformation Matrix
      Transform3D<> F5(sp5, sr5.toRotation3D()); Transform3D<> T5(ep5, er5.toRotation3D());

      //--------------------------------------------------------------------------------------------

      Transform3D<> Sp[] = {F1,F2,F3,F4,F5};
      Transform3D<> Ep[] = {T1,T2,T3,T4,T5};

    /** Most easy way: uses default parameters based on given device
        sampler: QSampler::makeUniform(device)
        metric: PlannerUtil::normalizingInfinityMetric(device->getBounds())
        extend: 0.05 */
    //QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(constraint, device, RRTPlanner::RRTConnect);

    /** More complex way: allows more detailed definition of parameters and methods */
    QSampler::Ptr sampler = QSampler::makeConstrained(QSampler::makeUniform(device),constraint.getQConstraintPtr());
    QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();
    QConstraint::Ptr Qconstraint = QConstraint::make(&detector, device, state);

    //double extend = 0.05;
    double extend = 0.05;

    QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(constraint, sampler, metric, extend, RRTPlanner::RRTConnect);


    //Input Initial position Trasformation matrix
//    Vector3D<> V0(0.1, -0.17, 0.98);
//    RPY<> R0(-2.9, -0.0, -1.5);
//    Transform3D<> FROM(V0, R0.toRotation3D()); //POSITION FROM TRANSFORMATION MATRIX

//    //Input Goal position Transformation matrix
//    Vector3D<> V1(0.08, -0.76, 0.40);
//    RPY<> R1(3.0, 0.1, -3.0);
//    Transform3D<> TO(V1, R1.toRotation3D()); //POSITION TO TRANSFORMATION MATRIX




    //Call RRT planner
    //planner->query(from,to,path,MAXTIME); //run planner
    int i=0; int k;
for (k=0;k<100;k++){
    i=0;
    while (i< 5){
        /** PRM PLANNER*/
        QPath path;
        Timer t;
        t.resetAndResume();

        Q from = inverseKinematics(device, sdevice, state, Sp[i] );
        Q to = inverseKinematics(device, sdevice, state, Ep[i]);

        if (!checkCollisions(device, state, detector, from))
            return 0;
        if (!checkCollisions(device, state, detector, to))
            return 0;
        if (OUT != 11)
            cout << "Planning from " << from << " to " << to << endl;

    //    PRMPlanner* prm = new PRMPlanner(device.get(), state, &detector, extend); //input device as (rw::models::Device*)
    //    PRMPlanner* prm = new PRMPlanner(Qconstraint, sampler, 0.1, device, state); //input device as (rw::models::Device*)
    //    prm->setCollisionCheckingStrategy(PRMPlanner::LAZY);
    //    prm->setNeighSearchStrategy(PRMPlanner::BRUTE_FORCE);
    //    prm->setShortestPathSearchStrategy(PRMPlanner::A_STAR);
    //    prm->buildRoadmap(10000);
        //Call PRM planner
        //prm->query(from,to,path,MAXTIME);
        planner->query(from,to,path,MAXTIME); //run planner

        //LinearInterpolate Path
        //path.size() == 2;
        //rw::math::Q start_configuration = path.at(0);
        //rw::math::Q end_configuration = path.at(1);

        // replace the path with an interpolated path
        //path = linearInterpolatedPath(start_configuration, end_configuration);

        // replace the path with an Optimized path
        //rwlibs::pathoptimization::PathLengthOptimizer::QList PathLengthOptimizer;
        //PathLengthOptimizer = rwlibs::pathoptimization::PathLengthOptimizer(constraint,metric);
        //path = PathLengthOptimizer.partialShortCut(path);
        t.pause();
        if (OUT!=11){
        cout << "Path of length " << path.size() << " found in " << t.getTime() << " seconds." << endl;
        if (t.getTime() >= MAXTIME) {
            cout << "Notice: max time of " << MAXTIME << " seconds reached." << endl;
        }

        for (QPath::iterator it = path.begin(); it < path.end(); it++) {
            cout << *it << endl;
        }
} else{
        if (t.getTime() >= MAXTIME) {
                cout << "30;" << "3;" << "5;"<< "-1" << endl;
            }
        else
                cout << t.getTime() << ";" << getTdistance(path,wc,device,state) << ";" << path.size() << endl;
}

        //delete(prm);
        //delete(path);
        i++;
    }

}
    cout << "Program done." << endl;
    return 0;

}
