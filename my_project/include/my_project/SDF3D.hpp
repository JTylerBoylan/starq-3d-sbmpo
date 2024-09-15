#ifndef SBMPO_SDF_3D_HPP_
#define SBMPO_SDF_3D_HPP_

#include <fstream>
#include <sstream> // std::stringstream
#include <string>
#include <vector>

namespace my_namespace
{

    class SDF3D
    {
    public:
        SDF3D()
        {
        }

        SDF3D(const int sizeX, const int sizeY, const int sizeZ, const float resX, const float resY, const float resZ)
            : sizeX_(sizeX), sizeY_(sizeY), sizeZ_(sizeZ), resX_(resX), resY_(resY), resZ_(resZ)
        {
            grid_ = std::vector<float>(sizeX * sizeY * sizeZ);
        }

        float getDistance(const float x, const float y, const float z)
        {
            const int xidx = x / resX_;
            const int yidx = y / resY_;
            const int zidx = z / resZ_;
            if (xidx < 0 || xidx >= sizeX_ ||
                yidx < 0 || yidx >= sizeY_ ||
                zidx < 0 || zidx >= sizeZ_)
                return 0.0;
            const int idx = zidx * (sizeX_ * sizeY_) + yidx * sizeX_ + xidx;
            return grid_[idx];
        }

        void setDistance(const int idx, const float dist)
        {
            grid_[idx] = dist;
        }

        void setDistance(const int xidx, const int yidx, const int zidx, const float dist)
        {
            const int idx = zidx * (sizeX_ * sizeY_) + yidx * sizeX_ + xidx;
            setDistance(idx, dist);
        }

        static SDF3D fromFile(const std::string file_path)
        {
            std::ifstream file;
            file.open(file_path);

            std::string sizes;
            std::getline(file, sizes);
            std::stringstream ss(sizes);

            std::string val;
            std::getline(ss, val, ',');
            int sizeX = std::stoi(val);
            std::getline(ss, val, ',');
            int sizeY = std::stoi(val);
            std::getline(ss, val, ',');
            int sizeZ = std::stoi(val);

            std::string ress;
            std::getline(file, ress);
            std::stringstream ssr(ress);

            std::getline(ssr, val, ',');
            float resX = std::stof(val);
            std::getline(ssr, val, ',');
            float resY = std::stof(val);
            std::getline(ssr, val, ',');
            float resZ = std::stof(val);

           SDF3D grid(sizeX, sizeY, sizeZ, resX, resY, resZ);

            float distval;
            for (int zi = 0; zi < sizeZ; zi++)
            {
                for (int yi = 0; yi < sizeY; yi++)
                {
                    std::string row;
                    std::getline(file, row);
                    std::stringstream ssrow(row);
                    for (int xi = 0; xi < sizeX; xi++)
                    {
                        std::getline(ssrow, val, ',');
                        distval = std::stof(val);
                        grid.setDistance(xi, yi, zi, distval);
                    }
                }
            }

            return grid;
        }

    private:
        int sizeX_, sizeY_, sizeZ_;
        float resX_, resY_, resZ_;
        std::vector<float> grid_;
    };

}

#endif