#include <random>
#include <limits>
#include <armadillo>
#include <kukadu/kukadu.hpp>

/*

using namespace std;
using namespace arma;
using namespace kukadu;

#define INVASIONGAMEREWARD_H_LEFT_PERCEPT 0
#define INVASIONGAMEREWARD_H_RIGHT_PERCEPT 1

#define INVASIONGAMEREWARD_H_LEFT_ACTION 0
#define INVASIONGAMEREWARD_H_RIGHT_ACTION 1

#define INVASIONGAMEREWARD_REW 1.0

class InvasionGameReward : public Reward {

private:

    int currentTimeStep;
    std::uniform_int_distribution<int> intDist;
    std::shared_ptr<std::vector<std::shared_ptr<PerceptClip> > > perceptClips;

protected:

    double computeRewardInternal(std::shared_ptr<PerceptClip> providedPercept, std::shared_ptr<ActionClip> takenAction) {

        int perceptId = providedPercept->getPerceptId();
        int actionId = takenAction->getActionId();
        if(++currentTimeStep < 250)
            return INVASIONGAMEREWARD_REW * (1 - abs(perceptId - actionId));
        else
            return INVASIONGAMEREWARD_REW * (abs(perceptId - actionId));

    }

public:

    InvasionGameReward(std::shared_ptr<std::mt19937> generator, bool collectPrevRewards) : Reward(generator, collectPrevRewards) {
        currentTimeStep = 0;
        intDist = std::uniform_int_distribution<int>(0, 1);
    }

    int getDimensionality() {
        return 1;
    }

    std::shared_ptr<PerceptClip> generateNextPerceptClip(int immunity) {
        return perceptClips->at(intDist(*generator));
    }

    std::shared_ptr<std::vector<std::shared_ptr<ActionClip> > > generateActionClips() {
        std::shared_ptr<vector<std::shared_ptr<ActionClip> > > actionClips = std::shared_ptr<vector<std::shared_ptr<ActionClip> > >(new vector<std::shared_ptr<ActionClip> >());
        actionClips->push_back(std::shared_ptr<ActionClip>(new ActionClip(INVASIONGAMEREWARD_H_LEFT_ACTION, 1, "left action", generator)));
        actionClips->push_back(std::shared_ptr<ActionClip>(new ActionClip(INVASIONGAMEREWARD_H_RIGHT_ACTION, 1, "right action", generator)));
        return actionClips;
    }

    std::shared_ptr<std::vector<std::shared_ptr<PerceptClip> > > generatePerceptClips() {
        auto defIm = ProjectiveSimulator::PS_DEFAULT_IMMUNITY;
        std::shared_ptr<vector<std::shared_ptr<PerceptClip> > > perceptClips = std::shared_ptr<vector<std::shared_ptr<PerceptClip> > >(new vector<std::shared_ptr<PerceptClip> >());
        perceptClips->push_back(std::shared_ptr<PerceptClip>(new PerceptClip(INVASIONGAMEREWARD_H_LEFT_PERCEPT, "left sign", generator, std::shared_ptr<vector<int> >(new vector<int>({INVASIONGAMEREWARD_H_LEFT_PERCEPT})), defIm)));
        perceptClips->push_back(std::shared_ptr<PerceptClip>(new PerceptClip(INVASIONGAMEREWARD_H_RIGHT_PERCEPT, "right sign", generator, std::shared_ptr<vector<int> >(new vector<int>({INVASIONGAMEREWARD_H_RIGHT_PERCEPT})), defIm)));
        return std::shared_ptr<std::vector<std::shared_ptr<PerceptClip> > >(this->perceptClips = perceptClips);
    }

};
*/
int main(int argc, char** args) {
/*
    ros::init(argc, args, "kukadu_projective_simulation_demo"); ros::NodeHandle node; sleep(1);
    ros::AsyncSpinner spinner(10); spinner.start();

    auto& storage = StorageSingleton::get();
    ModuleUsageSingleton::get().stopStatisticsModule();

    auto timePoint1 = chrono::system_clock::now();
    auto dtn = timePoint1.time_since_epoch();
    int seed = dtn.count();
    auto generator = make_shared<std::mt19937>(seed);
    auto invGameReward = make_shared<InvasionGameReward>(generator, false);

    auto originalMode = ProjectiveSimulator::PS_USE_ORIGINAL;
    auto ps = make_shared<ProjectiveSimulator>(invGameReward, generator, 0.2, originalMode, false);

    for(int i = 0; i < 5000; ++i) {
        ps->performRandomWalk();
        ps->performRewarding();
    }

    TreeDrawer t(1024, 768);
    t.drawTree(ps);
    t.waitForEnter();
*/
    return EXIT_SUCCESS;

}
