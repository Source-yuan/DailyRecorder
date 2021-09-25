#pragma once

#include "state.hpp"
#include <Eigen/Eigen>

using namespace Eigen;

namespace libMultiRobotPlanning
{
    class Maper
    {
    protected:
        int GLX_SIZE, GLY_SIZE, GLZ_SIZE; // 网格尺寸
        int GLXYZ_SIZE, GLYZ_SIZE;        //网格间接数据

        double resolution, inv_resolution; // 分辨率

        double gl_xl, gl_yl, gl_zl; //左下角和右上角
        double gl_xu, gl_yu, gl_zu; //的物理坐标值

    public:
        Maper() {}

        uint8_t *data; //存储占据情况

        void initGridMap(double _resolution = 0.2, double _x_size = 50.0, double _y_size = 50.0, double _z_size = 5.0)
        {

            // 网格地图，左下角、右上角坐标（物理尺度）
            gl_xl = -_x_size / 2.0;
            gl_yl = -_y_size / 2.0;
            gl_zl = 0.0;

            gl_xu = +_x_size / 2.0;
            gl_yu = +_y_size / 2.0;
            gl_zu = _z_size;

            // 分辨率和逆分辨率
            resolution = _resolution;
            inv_resolution = 1.0 / _resolution;

            // 网格地图的长宽高（网格坐标），默认(250, 250, 25)
            GLX_SIZE = (int)(_x_size * inv_resolution);
            GLY_SIZE = (int)(_y_size * inv_resolution);
            GLZ_SIZE = (int)(_z_size * inv_resolution);

            // 间接网格数据
            GLYZ_SIZE = GLY_SIZE * GLZ_SIZE;
            GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE; // 立方体总网格数

            // 为data开辟内存，存储网格占据情况
            data = new uint8_t[GLXYZ_SIZE];
            memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t)); //初始为0，全部为非占据(free)
        };

        void setObs(const double coord_x, const double coord_y, const double coord_z)
        {
            if (coord_x < gl_xl || coord_y < gl_yl || coord_z < gl_zl ||
                coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu)
                return;
            int idx_x = static_cast<int>((coord_x - gl_xl) * inv_resolution);
            int idx_y = static_cast<int>((coord_y - gl_yl) * inv_resolution);
            int idx_z = static_cast<int>((coord_z - gl_zl) * inv_resolution);

            data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
        };
    };
}

using namespace libMultiRobotPlanning;
int main()
{

    Maper map;

    return 0;
}