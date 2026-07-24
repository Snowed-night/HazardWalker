/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "FSM/FSMState.h"

FSMState::FSMState(CtrlComponents *ctrlComp, FSMStateName stateName, std::string stateNameString)
            :_ctrlComp(ctrlComp), _stateName(stateName), _stateNameString(stateNameString){
    _lowCmd = _ctrlComp->lowCmd;
    _lowState = _ctrlComp->lowState;
}

long long FSMState::getRosTime()
{
    return _ctrlComp->ioInter->current_time;
}

long long  FSMState::getTime()
{
    if (real == false)
    {
        return getRosTime();
    }
    else
    {
        return getSystemTime();
    }
}

void FSMState::wait(long long startTime, long long waitTime){
    if (real == false)
    {
        rosAbsoluteWait(startTime,waitTime);
    }
    else
    {
        absoluteWait(startTime,waitTime);
    }
}

void FSMState::rosAbsoluteWait(long long startTime, long long waitTime){
    overtime = 0;
    if(getRosTime() - startTime > waitTime){
        std::cout << "[WARNING] The waitTime=" << waitTime << " of function absoluteWait is not enough!" << std::endl
        << "The program has already cost " << getSystemTime() - startTime << "us." << std::endl;
    }
    while((getRosTime() - startTime < waitTime) && (overtime < OVERTIME)){

        // std::cout << "getRosTime()" << getRosTime() << std::endl;
        // std::cout << "startTime" << startTime << std::endl;
        // std::cout << "wait" << std::endl;
        // std::cout << "overtime" <<  overtime << std::endl;
        usleep(50);
        overtime += 50;
    }

}

//设置cmd_vel的回调函数，将move_base转化为
void FSMState::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg){
   if (msg) {
        std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
        if (!std::isfinite(msg->linear.x) ||
            !std::isfinite(msg->linear.y) ||
            !std::isfinite(msg->angular.z)) {
            current_cmd_vel_.linear_x = 0.0;
            current_cmd_vel_.linear_y = 0.0;
            current_cmd_vel_.angular_z = 0.0;
            current_cmd_vel_.valid = false;
            ROS_WARN_THROTTLE(1.0, "[CMD_VEL_REJECTED] non-finite command");
            return;
        }
        current_cmd_vel_.linear_x = msg->linear.x;
        current_cmd_vel_.linear_y = msg->linear.y;
        current_cmd_vel_.angular_z = msg->angular.z;
        current_cmd_vel_.valid = true;
        current_cmd_vel_.received_at = std::chrono::steady_clock::now();
        ROS_INFO_STREAM_THROTTLE(
            1.0,
            "[CMD_VEL_RX] x=" << current_cmd_vel_.linear_x
            << " y=" << current_cmd_vel_.linear_y
            << " yaw=" << current_cmd_vel_.angular_z
        );
    }
    // std::cout << "cmd_vel_linear_x"<< this->current_cmd_vel_.linear_x<< std::endl;
    // std::cout << "cmd_vel_linear_y"<< this->current_cmd_vel_.linear_y<< std::endl;
    // std::cout << "cmd_vel_angular_z"<< this->current_cmd_vel_.angular_z<< std::endl;
}


