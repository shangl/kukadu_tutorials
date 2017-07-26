#include <kukadu/kukadu.hpp>

using namespace std;
using namespace arma;
using namespace kukadu;

void publishKinectTransform();

int main(int argc, char** args) {

    ros::init(argc, args, "kukadu_artracker_demo"); ros::NodeHandle node; sleep(1);
    ros::AsyncSpinner spinner(10); spinner.start();

    auto& storage = StorageSingleton::get();

    thread transformThread(publishKinectTransform);

    auto kinect = make_shared<Kinect>(storage, node, true);
    //auto kinect = HardwareFactory::get().loadHardware("camera");
    kinect->install();
    kinect->start();

    auto localizerSkill = make_shared<LocalizeObject>(storage, KUKADU_DYNAMIC_POINTER_CAST<Kinect>(kinect));
    try { localizerSkill->createSkillFromThis("localize_object"); } catch(KukaduException& ex) {}

    localizerSkill->execute();

    auto& vis = VisualizerSingleton::get();
    vis.startWindow();

    vis.drawLine("coordinateFrameX", 0, 0, 0, 2, 0, 0);
    vis.drawLine("coordinateFrameY", 0, 0, 0, 0, 2, 0);
    vis.drawLine("coordinateFrameZ", 0, 0, 0, 0, 0, 1);
    vis.showPointCloud("pc", KUKADU_DYNAMIC_POINTER_CAST<Kinect>(kinect)->getCurrentColorPointCloud());

    cout << "press a key to end the program" << endl;
    getchar();

    KUKADU_DYNAMIC_POINTER_CAST<Kinect>(kinect)->stopSensing();

    storage.waitForEmptyCache();

    return EXIT_SUCCESS;

}

void publishKinectTransform() {

    cout << "publishing calibration" << endl;
    mat calibration(4, 4);
    calibration(0, 0) = -0.02081469;    calibration(0, 1) = -0.99929153;    calibration(0, 2) = 0.03135609;     calibration(0, 3) = 0.1;
    calibration(1, 0) = -0.99976864;    calibration(1, 1) = 0.02063391;     calibration(1, 2) = -0.00607739;    calibration(1, 3) = 0.563;
    calibration(2, 0) = 0.00542609;     calibration(2, 1) = -0.03147533;    calibration(2, 2) = -0.99948982;    calibration(2, 3) = 1.393;
    calibration(3, 0) = 0.0;            calibration(3, 1) = 0.0;            calibration(3, 2) = 0.0;            calibration(3, 3) = 1.0;
    tf::TransformBroadcaster br;
    auto transform = affineTransMatrixToTf(calibration);

    ros::Rate r(10);
    while(true) {
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_rgb_optical_frame", "origin"));
        r.sleep();
    }

}
