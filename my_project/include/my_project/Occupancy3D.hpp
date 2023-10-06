#ifndef SBMPO_OCCUPANCY_3D_HPP_
#define SBMPO_OCCUPANCY_3D_HPP_

#include <vector>

namespace my_namespace
{

    class Occupancy3D
    {
    public:
        Occupancy3D(const int sizeX, const int sizeY, const int sizeZ, const int resX, const int resY, const int resZ)
        : sizeX_(sizeX), sizeY_(sizeY), sizeZ_(sizeZ), resX_(resX), resY_(resY), resZ_(resZ)
        {
            grid_ = std::vector<bool>(sizeX*sizeY*sizeZ);
        }

    private:
        const int sizeX_, sizeY_, sizeZ_, resX_, resY_, resZ_;
        std::vector<bool> grid_;
    };

}

#endif