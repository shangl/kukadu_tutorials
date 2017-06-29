#include <armadillo>
#include <kukadu/kukadu.hpp>

using namespace std;
using namespace arma;
using namespace kukadu;

int main(int argc, char** args) {

    cout << "setting up ros node" << endl;
    ros::init(argc, args, "kukadu_controlqueue_demo"); ros::NodeHandle node; sleep(1);
    ros::AsyncSpinner spinner(10); spinner.start();

    int simulation = 1;

    for(int i = 1; i < argc; ++i) {
        KukaduTokenizer tok(args[i]);
        auto currentString = tok.next();
        KukaduTokenizer argTok(currentString, "=");
        auto currentArg = argTok.next();
        if(currentArg == "simulation")
            simulation = atoi(argTok.next().c_str());
    }

    StorageSingleton& storage = StorageSingleton::get();
    ModuleUsageSingleton::get().stopStatisticsModule();

    cout << "setting up control queue" << endl;
    auto hardwareFactory = HardwareFactory::get();
    auto realLeftQueue = dynamic_pointer_cast<KukieControlQueue>(hardwareFactory.loadHardware("kukie_left_arm"));
    realLeftQueue->install();

    cout << "starting queue" << endl;
    auto realLqThread = realLeftQueue->startQueue();

    cout << "switching to impedance mode if it is not there yet" << endl;
    if(realLeftQueue->getCurrentMode() != KukieControlQueue::KUKA_JNT_IMP_MODE) {
        realLeftQueue->stopCurrentMode();
        realLeftQueue->switchMode(KukieControlQueue::KUKA_JNT_IMP_MODE);
    }

    // joint point to point movement in order to go to the start position
    // remark: you only define where to go, not how to get there

    realLeftQueue->setStiffness(KukieControlQueue::KUKA_STD_XYZ_STIFF, KukieControlQueue::KUKA_STD_ABC_STIFF, 1.0, KukieControlQueue::KUKA_STD_CPMAXDELTA, KukieControlQueue::KUKA_STD_MAXFRC, KukieControlQueue::KUKA_STD_AXISMAXDELTATRQ);

    realLeftQueue->getPlanner()->setSpeed(1.0);
    //realLeftQueue->jointPtp(stdToArmadilloVec({-1.0, 1.0, -0.5, 0.0, 0.0, 0.0, 0.0}));
    realLeftQueue->jointPtp(stdToArmadilloVec({-1.5, 1.55, 2.33, -1.74, -1.85, 1.27, 0.71}));

    // retrieving the current joint state of the robot after the joint
    // point to point movement
    auto startState = realLeftQueue->getCurrentJoints().joints;

    while(true) {

        // execution a trajectory for the 3rd joint (i.e. rotation the arm)
        // here you also provide HOW to get to the target
        vec currentState;
        int movingJoint = 2;
        for(currentState = startState; currentState(movingJoint) < startState(movingJoint) + 0.5; currentState(movingJoint) += 0.003) {
            // sending a the next desired position
            realLeftQueue->move(currentState);
            // the queue has an intrinsic clock, so you can wait until the packet has been
            // submit in order to not send the positions too fast
            realLeftQueue->synchronizeToQueue(1);
        }

        for(; currentState(movingJoint) > startState(movingJoint); currentState(movingJoint) -= 0.003) {
            // sending a the next desired position
            realLeftQueue->move(currentState);
            // the queue has an intrinsic clock, so you can wait until the packet has been
            // submit in order to not send the positions too fast
            realLeftQueue->synchronizeToQueue(1);
        }

    }

    cout << "execution done" << endl;
    cout << "press a key to end the program" << endl;
    getchar();

    /****** done with moving? --> clean up everything and quit *******/

    // leaves the mode for robot movement
    realLeftQueue->stopCurrentMode();

    // stops the queue
    realLeftQueue->stopQueue();

    storage.waitForEmptyCache();

    return EXIT_SUCCESS;

}
