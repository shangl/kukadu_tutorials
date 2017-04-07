#include <limits>
#include <armadillo>
#include <kukadu/kukadu.hpp>

using namespace std;
using namespace arma;
using namespace kukadu;

int main(int argc, char** args) {

    ros::init(argc, args, "kukadu_projective_simulation_demo"); ros::NodeHandle node; sleep(1);
    ros::AsyncSpinner spinner(10); spinner.start();

    StorageSingleton& storage = StorageSingleton::get();
    ModuleUsageSingleton::get().stopStatisticsModule();

    GraphDrawer d;

    return 0;

}
