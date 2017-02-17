#include <limits>
#include <armadillo>
#include <kukadu/kukadu.hpp>

using namespace std;
using namespace arma;
using namespace kukadu;

int main(int argc, char** args) {

    bool compress = true;
    bool loadSkills = false;
    bool exportFunctionStatistics = true;

    string outFolder = "/tmp/data";
    string inputFolder;
    string distancesOutputFile = "/dev/null";

    for(int i = 1; i < argc; ++i) {
        KukaduTokenizer tok(args[i], "=");
        auto currTok = tok.next();
        if(currTok == "-c" && tok.next() == "false")
            compress = false;
        else if(currTok == "-f" && tok.next() == "false")
            exportFunctionStatistics = false;
        else if(currTok == "--help") {
            cout << "usage: rosrun kukadu_tutorials " << args[0] << " [-c={true | false}] [-f={true | false}] [-i=$input_path$ -o=$distances_output_file$] [$destinatio_path$]" << endl;
            return EXIT_FAILURE;
        } else if(currTok == "-i") {
            loadSkills = true;
            inputFolder = tok.next();
        } else if(currTok == "-o") {
            distancesOutputFile = tok.next();
        } else
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

    if(!loadSkills) {

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

    } else {

        cout << "loading data" << endl;
        auto importedSkill = exporter.loadExecutions(inputFolder);

        cout << "computing all pairwise distances" << endl;
        mat allDistances = exporter.computeAllNearestNeighbours(importedSkill.second);
        auto& successLabels = importedSkill.first;

        ofstream distancesFile;
        distancesFile.open(string(distancesOutputFile + "allDist").c_str());

        ofstream nearestNeighboursFile;
        nearestNeighboursFile.open(distancesOutputFile.c_str());

        vec nearestNeighbourDistances(allDistances.n_rows);
        vector<int> nearestNeibhbourIdx;
        cout << "find distance to best neighbour" << endl;
        for(int i = 0; i < allDistances.n_rows; ++i) {

            int currentMinIdx = -1;
            double currentMinDistance = std::numeric_limits<double>::max();
            // find the best neighbour that is a successful one and not itself
            for(int j = 0; j < allDistances.n_rows; ++j) {
                if(allDistances(i, j) < currentMinDistance && i != j && successLabels.at(j) == 0) {
                    currentMinDistance = allDistances(i, j);
                    currentMinIdx = j;
                }

            }

            nearestNeighbourDistances(i) = currentMinDistance;
            nearestNeibhbourIdx.push_back(currentMinIdx);

            nearestNeighboursFile << currentMinIdx << "\t" << currentMinDistance << endl;

        }

        double distanceThresh = 100000.0;

        int totalFailureCount = 0;
        int totalSuccessCount = 0;

        int correctlyClassifiedFailure = 0;
        int correctlyClassifiedSuccess = 0;
        int misclassified = 0;
        for(int i = 0; i < nearestNeighbourDistances.n_elem; ++i) {

            if(!successLabels.at(i))
                ++totalSuccessCount;
            else
                ++totalFailureCount;

            if(nearestNeighbourDistances(i) > distanceThresh && successLabels.at(i) != 0)
                ++correctlyClassifiedFailure;
            else if(nearestNeighbourDistances(i) < distanceThresh && successLabels.at(i) == 0) {
                ++correctlyClassifiedSuccess;
            } else {
                ++misclassified;
            }
        }

        cout << "summary:" << endl;
        cout << "correctly classified failure: " << correctlyClassifiedFailure << " / " << totalFailureCount << " (" << ((double) correctlyClassifiedFailure / totalFailureCount) << ")" << endl;
        cout << "correctly classified success: " << correctlyClassifiedSuccess << " / " << totalSuccessCount << " (" << ((double) correctlyClassifiedSuccess / totalSuccessCount) << ")" << endl;
        cout << "total misclassified: " << misclassified << " / " << nearestNeighbourDistances.n_elem << " (" << ((double) misclassified / nearestNeighbourDistances.n_elem) << ")" << endl;

        distancesFile << allDistances << endl;
        distancesFile.close();

    }

    return 0;

}
