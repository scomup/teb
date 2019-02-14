
#include <chrono>
#include <thread>
#include <memory>
#include <memory>
#include "src/transform/timestamped_transform.h"
#include "src/core/types.h"
#include "src/core/teb_planner.h"
#include "src/top/log_io.h"

#include <QApplication>
#include "src/qtviewer/window.h"


using namespace teb_demo;
int main(int argc, char **argv)
{

    //std::vector<transform::TimestampedTransform2d> path;
    //std::vector<Eigen::Vector2d> obstacles;
    LogFile f("/home/liu/workspace/teb/build/path.txt");
    std::vector<teb_demo::Pose2d> paths;
    std::vector<teb_demo::Obst2d> obsts;

      //f.paths().push_back(Pose2d{0,
      //                      0,
      //                      0});
//


    //QApplication app(argc, argv);
    //MainWindow window;
    //window.show();
    //return app.exec();


    TEBPlanner teb(f.paths(), obsts);
    teb.slove();
    printf("OK!\n");



}
