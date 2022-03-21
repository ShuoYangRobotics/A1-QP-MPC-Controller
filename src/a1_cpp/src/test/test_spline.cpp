#include <cstdio>
#include <vector>

#include <Eigen/Dense>
#include "../utils/spline.h"
#include "../utils/Utils.h"

int main(int, char**) {
    std::vector<double> X = {0.1, 0.4, 1.2, 1.8, 2.0}; // must be increasing
    std::vector<double> Y = {0.1, 0.7, 0.6, 1.1, 0.9};

    tk::spline s(X,Y);
    double x=1.5, y=s(x), deriv=s.deriv(1,x);

    printf("spline at %f is %f with derivative %f\n", x, y, deriv);

    Eigen::Vector3d foot_pos_start(0.2,0.15,-0.33);
    Eigen::Vector3d foot_pos_final(0.25,0.16,-0.33);

    CubicSpineUtils cs_utils;
    cs_utils.set_foot_pos_curve(foot_pos_start, foot_pos_final);
    Eigen::MatrixXd interp_pos_rst(3,11);
    Eigen::MatrixXd interp_vel_rst(3,11);
    Eigen::MatrixXd interp_acc_rst(3,11);
    int i = 0;
    for (double t=0.0; t<=1.0; t += 0.1) {
        interp_pos_rst.col(i) = cs_utils.get_foot_pos_curve(t);
        interp_vel_rst.col(i) = cs_utils.get_foot_vel_curve(t);
        interp_acc_rst.col(i) = cs_utils.get_foot_acc_curve(t);
        i++;
    }
    std::cout<<"position interpolation"<<std::endl;
    std::cout<<interp_pos_rst<<std::endl;
    std::cout<<"vel interpolation"<<std::endl;
    std::cout<<interp_vel_rst<<std::endl;
    std::cout<<"acc interpolation"<<std::endl;
    std::cout<<interp_acc_rst<<std::endl;

}