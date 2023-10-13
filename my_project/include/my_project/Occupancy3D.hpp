#ifndef SBMPO_OCCUPANCY_3D_HPP_
#define SBMPO_OCCUPANCY_3D_HPP_

#include <fstream>
#include <sstream> // std::stringstream
#include <string>
#include <vector>

namespace my_namespace
{

    class Occupancy3D
    {
    public:
        Occupancy3D()
        {
        }

        Occupancy3D(const int sizeX, const int sizeY, const int sizeZ, const float resX, const float resY, const float resZ)
            : sizeX_(sizeX), sizeY_(sizeY), sizeZ_(sizeZ), resX_(resX), resY_(resY), resZ_(resZ)
        {
            grid_ = std::vector<bool>(sizeX * sizeY * sizeZ);
        }

        bool getOccupancy(const float x, const float y, const float z)
        {
            const int xidx = x / resX_;
            const int yidx = y / resY_;
            const int zidx = z / resZ_;
            if (xidx < 0 || xidx >= sizeX_ ||
                yidx < 0 || yidx >= sizeY_ ||
                zidx < 0 || zidx >= sizeZ_)
                return false;
            const int idx = zidx * (sizeX_ * sizeY_) + yidx * sizeX_ + xidx;
            return grid_[idx];
        }

        void setOccupancy(const int idx, const bool occ)
        {
            grid_[idx] = occ;
        }

        void setOccupancy(const int xidx, const int yidx, const int zidx, const bool occ)
        {
            const int idx = zidx * (sizeX_ * sizeY_) + yidx * sizeX_ + xidx;
            setOccupancy(idx, occ);
        }

        static Occupancy3D fromFile(const std::string file_path)
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

            Occupancy3D grid(sizeX, sizeY, sizeZ, resX, resY, resZ);

            int occval;
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
                        occval = std::stoi(val);
                        grid.setOccupancy(xi, yi, zi, occval);
                    }
                }
            }

            return grid;
        }

    private:
        int sizeX_, sizeY_, sizeZ_;
        float resX_, resY_, resZ_;
        std::vector<bool> grid_;
    };

}

#endif