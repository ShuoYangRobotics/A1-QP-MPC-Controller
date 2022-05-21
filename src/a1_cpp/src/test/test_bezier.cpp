//
// Created by shuoy on 11/18/21.
//

#include <cstdio>
#include <vector>

#include <Eigen/Dense>
#include "../utils/Utils.h"

int main(int, char**) {

    Eigen::Vector3d foot_pos_start(0.2,0.15,-0.33);
    Eigen::Vector3d foot_pos_final(0.25,0.16,-0.33);

    BezierUtils bs_utils;
    Eigen::MatrixXd interp_pos_rst(3,11);
    int i = 0;
    for (double t=0.0; t<=1.0; t += 0.1) {
        interp_pos_rst.col(i) = bs_utils.get_foot_pos_curve(t,foot_pos_start, 
        foot_pos_final,0);
        i++;
    }
    std::cout<<"position interpolation"<<std::endl;
    std::cout<<interp_pos_rst<<std::endl;

}