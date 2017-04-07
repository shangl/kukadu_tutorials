#include <kukadu/kukadu.hpp>

using namespace std;
using namespace kukadu;

int main(int argc, char** args) {

    if(argc != 2) {
        cout << "usage: rosrun kukadu_tutorials libsvm $PATH_TO_DATA$" << endl;
        return EXIT_FAILURE;
    }

    cout << "creating svm instance" << endl;
    LibSvm sv(resolvePath(string(args[1])));
    cout << "training the svm " << endl;
    sv.train();

    return EXIT_SUCCESS;

}
