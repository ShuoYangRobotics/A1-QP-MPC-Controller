//
// Created by shuoy on 1/25/22.
//
#include <iostream>
#include <ctime>

#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>

#include "../A1CtrlStates.h"
#include "../A1Params.h"
#include "../utils/Utils.h"

int main(int, char **) {
    srand((unsigned int) time(0));
    // random rotation
    Eigen::Quaterniond quat = Eigen::Quaterniond::UnitRandom();
    std::cout << "quat " << quat.coeffs().transpose() << std::endl;
    // convert to euler using our utility function
    Eigen::Vector3d euler  = Utils::quat_to_euler(quat);  // roll, pitch, yaw
    std::cout << "euler using our function " << euler.transpose() << std::endl;
    double roll = euler(0), pitch = euler(1), yaw  = euler(2);
    // convert to euler using Eigen function
    Eigen::Vector3d euler2  = quat.toRotationMatrix().eulerAngles(2, 1, 0); // yaw, pitch, roll
    std::cout << "euler2 using eigen function" << euler2.transpose() << std::endl;
    double roll2 = euler(2), pitch2 = euler(1), yaw2  = euler(0);
    // eigen's euler angle is not good because it's range is very strange


    // next let's see how to assemble roll pitch yaw to rotation matrix

    std::cout << "rotation matrix from quat" << std::endl << quat.toRotationMatrix() << std::endl;

    Eigen::Quaterniond quat2  = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
                                * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                                  *Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    std::cout << "rotation matrix from r p y  is " << std::endl << quat2.toRotationMatrix() << std::endl;
    // quat2 should equals to quat

    Eigen::Quaterniond quat3  = Eigen::AngleAxisd(yaw2, Eigen::Vector3d::UnitZ())
                                * Eigen::AngleAxisd(pitch2, Eigen::Vector3d::UnitY())
                                *Eigen::AngleAxisd(roll2, Eigen::Vector3d::UnitX());
    std::cout << "rotation matrix from r2 p2 y2  is " << std::endl << quat3.toRotationMatrix() << std::endl;
    // again, eigen's euler angle cannot be assembled back to the original matrix


    double cos_yaw = cos(yaw);
    double sin_yaw = sin(yaw);
    double cos_pitch = cos(pitch);
    double tan_pitch = tan(pitch);

    Eigen::Matrix3d ang_vel_to_rpy_rate;

    ang_vel_to_rpy_rate << cos_yaw / cos_pitch, sin_yaw / cos_pitch, 0,
            -sin_yaw, cos_yaw, 0,
            cos_yaw * tan_pitch, sin_yaw * tan_pitch, 1;

    std::cout << "ang_vel_to_rpy_rate  " << std::endl << ang_vel_to_rpy_rate << std::endl;

    return 0;
}
