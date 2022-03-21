//
// Created by shuoy on 11/1/21.
//

#include <cstdio>
#include <vector>
#include <ros/ros.h>
#include "unitree_legged_msgs/MotorCmd.h"
#include "unitree_legged_msgs/MotorState.h"

int main(int, char**) {
    unitree_legged_msgs::MotorCmd lastCmd;
    unitree_legged_msgs::MotorState lastState;
    lastState.q = 9;
    return 0;
}
