
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
    LogFile f("/home/liu/workspace/teb/script/path.txt");
    YAML::Node config = YAML::LoadFile("/home/liu/workspace/teb/config/sample.yaml");
    OptimalPlanner teb(&config);
    for (auto &pose : f.paths())
    {
        teb.addPose(pose);
    }
    for (auto &obst : f.obsts())
    {
        teb.addObstacle(obst);
    }
    teb.solve();
    teb.report();
}
