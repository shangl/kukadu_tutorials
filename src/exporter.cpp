#include <armadillo>
#include <kukadu/kukadu.hpp>

using namespace std;
using namespace arma;
using namespace kukadu;

int main(int argc, char** args) {

    bool exportFunctionStatistics = true;
    bool compress = true;
    string outFolder = "/tmp/data";

    for(int i = 1; i < argc; ++i) {
        KukaduTokenizer tok(args[i], "=");
        auto currTok = tok.next();
        if(currTok == "-c" && tok.next() == "false")
            compress = false;
        else if(currTok == "-f" && tok.next() == "false")
            exportFunctionStatistics = false;
        else
            outFolder = string(args[i]);
    }

    ros::init(argc, args, "kukadu_skillexporter_demo"); ros::NodeHandle node; sleep(1);
    ros::AsyncSpinner spinner(10); spinner.start();

    StorageSingleton& storage = StorageSingleton::get();
    ModuleUsageSingleton::get().stopStatisticsModule();

    auto& sensorSingleton = SensorStorageSingleton::get();

    auto leftHand = make_shared<KukieHand>(storage, node, "simulation", "left");
    leftHand->install();

    auto rightHand = make_shared<KukieHand>(storage, node, "simulation", "right");
    rightHand->install();

    auto leftArm = make_shared<KukieControlQueue>(storage, "robinn", "simulation", "left_arm", node);
    leftArm->install();

    auto rightArm = make_shared<KukieControlQueue>(storage, "robinn", "simulation", "right_arm", node);
    rightArm->install();

    sensorSingleton.registerHardware(leftHand);
    sensorSingleton.registerHardware(rightHand);
    sensorSingleton.registerHardware(leftArm);
    sensorSingleton.registerHardware(rightArm);

    SkillExporter exporter(storage);

    auto skills = exporter.getSkills();

    // list all skills
    cout << "select a skill to export: " << endl;
    for(auto skill : skills) {
        int skillId = skill.first;
        auto hardwareList = exporter.getSkillHardware(skillId);
        cout << skillId << " - " << skill.second << " (";
        for(int i = 0; i < hardwareList.size(); ++i) {
            cout << hardwareList.at(i);
            if(i < (hardwareList.size() - 1))
                cout << ", ";
        }
        cout << ")" << endl;
    }

    cout << "selection: ";
    int skillSelection;
    cin >> skillSelection;

    // list all executions of the selected skill
    int executionNum = 0;
    cout << "select the start execution:" << endl;
    auto executions = exporter.getSkillExecutions(skillSelection);
    for(auto& execution : executions) {
        long long int startTime = get<0>(execution);
        long long int endTime = get<1>(execution);
        bool succ = get<2> (execution);
        cout << executionNum++ << " - " << startTime << " --> " << endTime << ": " << ((succ) ? "successful" : "failed") << endl;
    }
    cout << "seclection: ";
    int executionSelection = 0;
    cin >> executionSelection;

    exporter.exportSkillExecutions(skillSelection, get<0>(executions.at(executionSelection)), get<1>(executions.back()), outFolder, exportFunctionStatistics, compress);

    return 0;

}
