
#include <chrono>
#include <thread>
#include <memory>
#include <memory>
#include "src/teb/optimal_planner.h"
#include "src/top/log_io.h"
#include "src/teb/obstacles.h"
#include "src/teb/robot_footprint_model.h"
#include "yaml-cpp/yaml.h"


using namespace teb_demo;
int main(int argc, char **argv)
{
    if(argc != 3)
    {
        printf("the first argument is the path of path file\n");
        printf("the second argument is the path of yaml file\n");
        return 0;
    }
    printf("=========================\n");
    LogFile f(argv[1]);
    YAML::Node config = YAML::LoadFile(argv[2]);
    OptimalPlanner teb(&config);
    for (auto &pose : f.paths())
    {
        teb.addPose(pose);
    }
    for (auto &obst : f.obsts())
    {
        teb.addObstacle(obst);
    }
    teb.getCloestDist();
    teb.solve();
    teb.report();
    teb.getCloestDist();
    return 0;
}
