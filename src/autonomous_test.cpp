#include <limits>
#include <armadillo>
#include <kukadu/kukadu.hpp>

using namespace std;
using namespace arma;
using namespace kukadu;

int main(int argc, char** args) {

    bool loadSkills = false;

    string inputFolder;

    for(int i = 1; i < argc; ++i) {
        KukaduTokenizer tok(args[i], "=");
        auto currTok = tok.next();
        if(currTok == "--help") {
            cout << "usage: rosrun kukadu_tutorials " << args[0] << " [-i=$input_path$]" << endl;
            return EXIT_FAILURE;
        } else if(currTok == "-i") {
            loadSkills = true;
            inputFolder = tok.next();
        }
    }

    ros::init(argc, args, "kukadu_skillexporter_demo"); ros::NodeHandle node; sleep(1);
    ros::AsyncSpinner spinner(10); spinner.start();

    StorageSingleton& storage = StorageSingleton::get();
    ModuleUsageSingleton::get().stopStatisticsModule();

    SkillExporter exporter(storage);

    if(loadSkills) {

        cout << "loading data" << endl;
        std::vector<long long int> startTimes;
        std::vector<long long int> endTimes;
        long long int timeStep;
        auto importedSkill = exporter.loadExecutions(inputFolder, startTimes, endTimes, timeStep);

        AutonomousTester tester(storage, "simple_grasp", {}, importedSkill, startTimes, endTimes, timeStep);
        tester.testSkill("simple_grasp");

        /*
        while(true) {
            int executionIdx = 0;
            cout << "insert the skill execution index with which you want to test: ";
            cin >> executionIdx;
            tester.computeFailureProb("simple_grasp", importedSkill.second.at(executionIdx));
        }
        */

    }

    return 0;

}
