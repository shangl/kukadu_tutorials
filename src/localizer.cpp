#include <kukadu/kukadu.hpp>

using namespace std;
using namespace arma;
using namespace kukadu;

int main(int argc, char** args) {

    ros::init(argc, args, "kukadu_artracker_demo"); ros::NodeHandle node; sleep(1);
    ros::AsyncSpinner spinner(10); spinner.start();

    HardwareFactory::get().setSimulation(false);

    auto& storage = StorageSingleton::get();

    auto kinect = HardwareFactory::get().loadHardware("camera");
    kinect->install();
    kinect->start();

    auto& vis = VisualizerSingleton::get();
    vis.startWindow();
    vis.drawLine("coordinateFrameX", 0, 0, 0, 2, 0, 0);
    vis.drawLine("coordinateFrameY", 0, 0, 0, 0, 2, 0);
    vis.drawLine("coordinateFrameZ", 0, 0, 0, 0, 0, 1);

    vis.drawLine("center", 0.7, 0.3, 0.5, 0.7, 0.3, 1.3);
    vis.drawLine("object", 0.7, 0.3, 0.5, 0.2, 0.63, 0.5);
    vis.drawLine("objectcenter", 0.2, 0.63, 0.5, 0.2, 0.63, 0.0);

    vis.showPointCloud("pc", KUKADU_DYNAMIC_POINTER_CAST<Kinect>(kinect)->getCurrentColorPointCloud());

    auto kukiearm = HardwareFactory::get().loadHardware("kukie_left_arm");
    kukiearm->install();
    kukiearm->start();

    KUKADU_DYNAMIC_POINTER_CAST<ControlQueue>(kukiearm)->jointPtp({-1.5, 1.56, 2.33, -1.74, -1.85, 1.27, 0.71});

    auto localizerSkill = make_shared<LocalizeObject>(storage, KUKADU_DYNAMIC_POINTER_CAST<Kinect>(kinect));
    try { localizerSkill->createSkillFromThis("localize_object"); } catch(KukaduException& ex) {}

    localizerSkill->execute();

    auto position = PoseEstimatorFactory::get().getPoseFor("something");
/*
    position.pose.position.x = position.pose.position.x - 0.43;
    position.pose.position.y = position.pose.position.y + 0.42;
    position.pose.position.z = position.pose.position.z - 0.03;
    position.pose.position.z = position.pose.position.z + 0.15;
    */
    position.pose.position.z = position.pose.position.z + 0.15;
    cout << position.pose.position.x << " " << position.pose.position.y << " " << position.pose.position.z << endl;
    tf::Quaternion rot = rpyToQuat(0.0, M_PI, 0.0);
    position.pose.orientation.x = rot.getX();
    position.pose.orientation.y = rot.getY();
    position.pose.orientation.z = rot.getZ();
    position.pose.orientation.w = rot.getW();

    auto kukiehand = HardwareFactory::get().loadHardware("kukiehand_left");
    kukiehand->install();
    kukiehand->start();

    auto openHand = SkillFactory::get().loadSkill("JointPtp", {kukiehand});
    auto openHandSkill = KUKADU_DYNAMIC_POINTER_CAST<JointPtp>(openHand);

    openHandSkill->setJoints({1.0443761317587406, -0.9809909670230643, 0.8357773148965317, -1.16191880779005, 1.1662613582635333, -1.164864130797623, 1.1698461062488343});
    openHandSkill->execute();

    auto graspSkill =  SkillFactory::get().loadSkill("CartesianPtp", {kukiearm});

    auto moveSkill = KUKADU_DYNAMIC_POINTER_CAST<CartesianPtp>(graspSkill);

    moveSkill->setCartesians(position.pose);
    moveSkill->execute();

    auto openHandTwo = SkillFactory::get().loadSkill("JointPtp", {kukiehand});
    auto openHandSkillTwo = KUKADU_DYNAMIC_POINTER_CAST<JointPtp>(openHandSkill);

    openHandSkillTwo->setJoints({1.0443761317587406, -0.19303613568962635, 0.3270285585631101, -0.17407614313821393, 0.4921333275819309, -0.22580197197676635, 0.5792149668019959});
    openHandSkillTwo->execute();

    position.pose.position.z += 0.1;

    auto graspSkillTwo =  SkillFactory::get().loadSkill("CartesianPtp", {kukiearm});

    auto moveSkillTwo = KUKADU_DYNAMIC_POINTER_CAST<CartesianPtp>(graspSkill);

    moveSkillTwo->setCartesians(position.pose);
    moveSkillTwo->execute();

    cout << "press a key to end the program" << endl;
    getchar();

    KUKADU_DYNAMIC_POINTER_CAST<Kinect>(kinect)->stopSensing();

    storage.waitForEmptyCache();

    return EXIT_SUCCESS;

}