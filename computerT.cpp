#include <iostream>
#include <Eigen/Core>

#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;


int main(int argc, char** argv)
{

    Eigen::Matrix3d R_ic;
    Eigen::Vector3d t_ic;

    R_ic << -0.01787795, -0.00545498,  0.9998253, 
            -0.99981167, -0.00745345, -0.01791837,
            0.00754989, -0.99995734, -0.0053207;

    t_ic<< 0.04043489, -0.00593765, -0.01431643;


    Eigen::Matrix3d R_cl;
    Eigen::Vector3d t_cl;

    // R_cl << -0.0401303, -0.99898, 0.0206851, -0.0199962, -0.0198947, -0.999602, 0.998994, -0.040528, -0.0191774;
    // t_cl << -0.006, 0.06, 0.015;
    // R_cl << -0.0202424,  -0.999678,  0.0153097, -0.00957396,  -0.0151184, -0.99984, 0.999749,  -0.0203858, -0.00926481;
    // t_cl << -0.085433, 0.0109158, -0.122115;

    R_cl << -0.0250049,    -0.999688,   0.00068552, 0.000937395, -0.000709336,    -0.999999, 0.999687,   -0.0250042,  0.000954885;
    t_cl << -0.00428288, 0.019991, -0.00969694;

    Eigen::Matrix3d R_il;
    Eigen::Vector3d t_il;

    R_il = R_ic * R_cl;
    t_il = R_ic * t_cl + t_ic;

    Eigen::Matrix3d R_li;
    Eigen::Vector3d t_li;

    R_li = R_il.inverse();
    t_li = -R_il.inverse()*t_il;

    cout<<"R_il: "<<R_il<<endl;
    cout<<"t_il: "<<t_il.transpose()<<endl;

    cout<<"R_li: "<<R_li<<endl;
    cout<<"t_li: "<<t_li.transpose()<<endl;

    Eigen::Vector3d euler_angles = R_cl.eulerAngles(2, 1, 0);
    cout<<"euler_angles: "<<euler_angles.transpose()<<endl;

    Eigen::Vector3d euler_angles_ = R_li.eulerAngles(2, 1, 0);
    cout<<"euler_angles_: "<<euler_angles_.transpose()<<endl;

    cout<<"-----------MVSEC database---------------"<<endl;

    Eigen::Matrix3d R_c0L;
    Eigen::Vector3d t_c0L;
    R_c0L << -0.01431797, -0.99923319,  0.03644205,
             -0.01489848, -0.03622854, -0.99923247,
             0.99978649, -0.01484991, -0.01436833;
    t_c0L << 0.04177696, -0.07663154, -0.13257184;

    Eigen::Matrix3d R_c2c1;
    Eigen::Vector3d t_c2c1;
    R_c2c1 << -0.9999194492806436, -0.011544126378795554, 0.005275234254147117,
              0.011675616860827782, -0.9996036981416507, 0.025614968913630146,
              0.004977441230681126, 0.02567449722346051, 0.9996579641412903;
    t_c2c1 << -0.11735831777218302, 0.05367704173973196, -0.007199602674672133;

    Eigen::Matrix3d R_c1c0;
    Eigen::Vector3d t_c1c0;
    R_c1c0 << 0.9998538711975195, 0.007121502935956221, -0.015540928133860692,
              -0.0069546143302057035, 0.9999178555872617, 0.010766402244230172,
              0.015616324498637743, -0.010656747801258783, 0.9998212660948196;
    t_c1c0 << -0.10020966289113746, -0.0004981408591306691, -0.0008688666914359259;


    Eigen::Matrix3d R_c2L;
    Eigen::Vector3d t_c2L;
    R_c2L = R_c2c1 * R_c1c0 * R_c0L;
    t_c2L = R_c2c1 * R_c1c0 * t_c0L + R_c2c1 * t_c1c0 + t_c2c1;

    cout<<"R_c2L = "<< R_c2L <<endl;
    cout<<"t_c2L = "<< t_c2L.transpose() <<endl;

    Eigen::Matrix3d R_c2b;
    Eigen::Vector3d t_c2b;
    R_c2b << 0.9999717314190615, -0.007438121416209933, 0.001100323844221122,
             0.00743200596269379, 0.9999574688631824, 0.005461295826837418,
             -0.0011408988276470173, -0.0054529638303831614, 0.9999844816472552;
    t_c2b << -0.03921656415229387, 0.00621263233002485, 0.0012210059575531885;

    Eigen::Matrix3d R_bc2;
    Eigen::Vector3d t_bc2;
    R_bc2 = R_c2b.transpose();
    t_bc2 = -R_bc2 * t_c2b;

    std::cout<<"R_bc2 = "<<R_bc2<<std::endl;
    std::cout<<"t_bc2 = "<<t_bc2.transpose()<<std::endl;

    R_c2L << 0.00774898, 0.998822, 0.0478988, 0.0480855,-0.0482171,0.997679, 0.998813,-0.00542768,-0.0484025;
    t_c2L << 0.117005, -0.0376154, -0.320574;

    Eigen::Matrix3d R_bL;
    Eigen::Vector3d t_bL;
    R_bL = R_bc2 * R_c2L;
    t_bL = R_bc2 * t_c2L + t_bc2;

    cout<<"R_bL = "<< R_bL <<endl;
    cout<<"t_bL = "<< t_bL.transpose() <<endl;

    cout<<"R_Lb = "<< R_bL.transpose() <<endl;
    cout<<"t_Lb = "<< (-R_bL.transpose() * t_bL).transpose() <<endl;

    return 0;
}