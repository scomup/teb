
#include <chrono>
#include <thread>
#include <memory>
#include <memory>
#include "src/teb/optimal_planner.h"
#include "src/top/log_io.h"
#include "src/teb/obstacles.h"
#include "src/teb/robot_footprint_model.h"

#include <QApplication>
#include "src/qtviewer/window.h"


using namespace teb_demo;
int main(int argc, char **argv)
{

    //std::vector<transform::TimestampedTransform2d> path;
    //std::vector<Eigen::Vector2d> obstacles;
    LogFile f("/home/liu/workspace/teb/path.txt");

    OptimalPlanner teb;
    for(auto &p : f.paths()){
        teb.addPose(p[0], p[1], p[2]);
    }
        for(auto &o : f.obsts()){
        teb.addObstacle(o[0], o[1]);
    }
    teb.solve();
    printf("OK!\n");



}
