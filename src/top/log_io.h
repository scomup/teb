#ifndef G2O_IO_H_
#define G2O_IO_H_

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

#include <sstream>
#include <map>


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
                double x = std::stof(v[1]);
                double y = std::stof(v[2]);
                double yaw = std::stof(v[3]);
                Eigen::Vector3d pose = Eigen::Vector3d(x, y, yaw);

                paths_.push_back(pose);
            }
            else if (v[0] == "OBST:")
            {
                std::vector< Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > obst;
                for (uint32_t i = 1; i < v.size(); i += 2)
                {
                    double x = std::stof(v[i]);
                    double y = std::stof(v[i + 1]);
                    Eigen::Vector2d point = Eigen::Vector2d(x, y);
                    obst.push_back(point);
                }
                obsts_.push_back(obst);
            }
            else
            {
                std::cout << "undefined: " << v[0] << "\n";
                continue;
            }
        }
    }

    //std::map<int, sample_carto::transform::Rigid3d>& nodes(){return nodes_;}
    std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& paths(){return paths_;}
    std::vector<std::vector< Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >>& obsts(){return obsts_;}

  private:
    std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > paths_;
    std::vector< std::vector< Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >> obsts_;
};

#endif // G2O_IO_H_


