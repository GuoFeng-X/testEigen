#include <iostream>
#include <Eigen/Core>

#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;


int main(int argc, char** argv)
{

    Eigen::Matrix3d R_ic;
    Eigen::Vector3d t_ic;

    R_ic << 0.00781297, -0.0042792, 0.99996,
            -0.999859, -0.014868, 0.00774856,
            0.0148343, -0.99988, -0.00439476;

    t_ic<< 1.1439, -0.312718, 0.726546;


    Eigen::Matrix3d R_cl;
    Eigen::Vector3d t_cl;

    R_cl << 7.027555e-03, -9.999753e-01, 2.599616e-05, -2.254837e-03, -4.184312e-05, -9.999975e-01, 9.999728e-01, 7.027479e-03, -2.255075e-03;
    t_cl << -7.137748e-03, -7.482656e-02, -3.336324e-01;

    Eigen::Matrix3d R_il;
    Eigen::Vector3d t_il;

    R_il = R_ic * R_cl;
    t_il = R_ic * t_cl + t_ic;

    cout<<"R_il: "<<R_il<<endl;
    cout<<"t_il: "<<t_il.transpose()<<endl;

    return 0;
}