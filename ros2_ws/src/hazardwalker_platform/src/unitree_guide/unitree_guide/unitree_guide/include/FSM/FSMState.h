/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef FSMSTATE_H
#define FSMSTATE_H

#include <string>
#include <iostream>
#include <chrono>
#include <cmath>
#include <mutex>
#include <unistd.h>
#include "control/CtrlComponents.h"
#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"
#include "common/enumClass.h"
#include "common/mathTools.h"
#include "common/mathTypes.h"
#include "common/timeMarker.h"
#include "interface/CmdPanel.h"

//添加ros节点，读取move_base
#include <ros/ros.h>
#include "geometry_msgs/Twist.h"

#define  OVERTIME 200000
class FSMState{

struct CmdVel {
    double linear_x = 0.0;
    double linear_y = 0.0;
    double angular_z = 0.0;
    bool valid = false;
    // 安全看门狗必须使用不受 /clock 暂停、回拨影响的单调时钟。
    std::chrono::steady_clock::time_point received_at;
};

public:
    FSMState(CtrlComponents *ctrlComp, FSMStateName stateName, std::string stateNameString);

    virtual void enter() = 0;
    virtual void run() = 0;
    virtual void exit() = 0;
    virtual FSMStateName checkChange() {return FSMStateName::INVALID;}

    FSMStateName _stateName;
    std::string _stateNameString;
    int real = false;
    long long getRosTime();
    long long getTime();
    void rosAbsoluteWait(long long startTime, long long waitTime);
    void wait(long long startTime, long long waitTime);


protected:
    CtrlComponents *_ctrlComp;
    FSMStateName _nextStateName;

    LowlevelCmd *_lowCmd;
    LowlevelState *_lowState;
    UserValue _userValue;

    uint32_t overtime = 0;


/*ros节点的私类变量*/
public:
    //cmd_vel callback function
    CmdVel current_cmd_vel_;
    // RL 推理线程读取速度的同时，ROS 主循环可能更新回调；互斥保护用于避免数据竞争。
    std::mutex cmd_vel_mutex_;
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);
    // 仅把新近收到的非零速度视为“需要行走”。零速度或断流时应回到固定站立，
    // 不让 RL 策略在静止时维持低趴姿态。
    bool hasFreshMotionCommand(double deadband = 0.02, double timeout_sec = 0.50);
    ros::NodeHandle nh;
    //声明订阅节点与发布节点
    ros::Subscriber Sub_;
    ros::Publisher Pub_;

};

#endif  // FSMSTATE_H
