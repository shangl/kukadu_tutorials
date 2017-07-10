#include <string>
#include <iostream>
#include <armadillo>
#include <kukadu/kukadu.hpp>

#include <QStyle>
#include <QStyleFactory>
#include <QtWidgets/QApplication>
#include <boost/program_options.hpp>


using namespace std;
using namespace arma;
using namespace kukadu;

int main(int argc, char** args) {
    ros::init(argc, args, "kukadu");
    ros::NodeHandle* node = new ros::NodeHandle();
    usleep(1e6);

    QApplication w(argc, args);
    w.setStyle(QStyleFactory::create("Fusion"));

    KukaduGraphical* g = new KukaduGraphical();
    g->show();

    return w.exec();
}

