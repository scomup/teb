#ifndef G2O_IO_H_
#define G2O_IO_H_

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

#include <sstream>
#include <map>

#include "src/core/types.h"

class LogFile
{
  public:
    std::vector<std::string> split(const std::string &str, char sep)
    {
        std::vector<std::string> v;
        std::stringstream ss(str);
        std::string buffer;
        while (std::getline(ss, buffer, sep))
        {
            v.push_back(buffer);
        }
        v.erase(
            std::remove_if(
                v.begin(), v.end(),
                [](std::string str) { return str == std::string(""); }),
            v.end());
        return v;
    }

    LogFile(std::string filename)
    {
        std::ifstream infile(filename);

        if (infile.fail())
        {
            std::cout << "Failed to open file: " << filename << std::endl;
            exit(0);
        }
        std::string line;
        while (std::getline(infile, line))
        {
            std::istringstream iss(line);
            auto v = split(line, ' ');
            if (v[0] == "PATH:")
            {
                double x = std::stod(v[1]);
                double y = std::stod(v[2]);
                double yaw = 0;
                paths_.push_back(teb_demo::Pose2d{x,y,yaw});
            }
            else if (v[0] == "OBST:")
            {
                double x = std::stod(v[1]);
                double y = std::stod(v[2]);
                obsts_.push_back(teb_demo::Obst2d{x,y});
            }
            else
            {
                std::cout << "undefined: " << v[0] << "\n";
                continue;
            }
        }
    }

    //std::map<int, sample_carto::transform::Rigid3d>& nodes(){return nodes_;}
    std::vector<teb_demo::Pose2d>& paths(){return paths_;}
    std::vector<teb_demo::Obst2d>& obsts(){return obsts_;}

  private:
    std::vector<teb_demo::Pose2d> paths_;
    std::vector<teb_demo::Obst2d> obsts_;
};

#endif // G2O_IO_H_


