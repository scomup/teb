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
                paths_.push_back({x,y,yaw});
            }
            else if (v[0] == "OBST:")
            {
                double x = std::stof(v[1]);
                double y = std::stof(v[2]);
                obsts_.push_back({x,y});
            }
            else
            {
                std::cout << "undefined: " << v[0] << "\n";
                continue;
            }
        }
    }

    //std::map<int, sample_carto::transform::Rigid3d>& nodes(){return nodes_;}
    std::vector<std::array<double,3>>& paths(){return paths_;}
    std::vector<std::array<double,2>>& obsts(){return obsts_;}

  private:
    std::vector<std::array<double,3>> paths_;
    std::vector<std::array<double,2>> obsts_;
};

#endif // G2O_IO_H_


