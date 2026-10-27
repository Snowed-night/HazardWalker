/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include <iostream>
#include <cstdlib>
#include <string>
#include "FSM/State_FixedStand.h"

State_FixedStand::State_FixedStand(CtrlComponents *ctrlComp)
                :FSMState(ctrlComp, FSMStateName::FIXEDSTAND, "fixed stand"){}

void State_FixedStand::enter(){
    // 固定站立也订阅速度，以便用户第一次按键时才切入 RL 行走状态。
    // 订阅者属于该状态对象；再次进入时赋值会替换旧订阅，不会叠加回调。
    Sub_ = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1,
        boost::bind(&FSMState::cmdVelCallback, this, _1));
    for(int i=0; i<4; i++){
        if(_ctrlComp->ctrlPlatform == CtrlPlatform::GAZEBO){
            _lowCmd->setSimStanceGain(i);
        }
        else if(_ctrlComp->ctrlPlatform == CtrlPlatform::REALROBOT){
            _lowCmd->setRealStanceGain(i);
        }
        _lowCmd->setZeroDq(i);
        _lowCmd->setZeroTau(i);
    }
    for(int i=0; i<12; i++){
        _lowCmd->motorCmd[i].q = _lowState->motorState[i].q;
        _startPos[i] = _lowState->motorState[i].q;
        _startPos_real[i] = _ctrlComp->ioInterFreeDog->low_state.motorState_free_dog[i].q;
    }
    // 必须从出生关节角平滑过渡到固定站姿。直接把 _percent 设为 1 会在解除
    // 物理暂停后的首周期瞬间跳变 12 个关节，A1 会被冲击掀翻；控制循环使用
    // 墙钟运行，即使 Gazebo 实时倍率较低，原有插值仍能在数秒内完成。
    _percent = 0.0f;
    _ctrlComp->setAllStance();
}

void State_FixedStand::run(){
    ros::spinOnce();
    _percent += (float)1/_duration;
    _percent = _percent > 1 ? 1 : _percent;
    for(int j=0; j<12; j++){
        _lowCmd->motorCmd[j].q = (1 - _percent)*_startPos[j] + _percent*_targetPos[j]; 
    }

    if (real == true){
        for(int j=0; j<12; j++){
            std::vector<double> joint{(1 - _percent)*_startPos_real[j] + \
                _percent*_targetPos[j], 0, 0, real_stand_p[j], real_stand_d[j]};
            _ctrlComp->ioInterFreeDog->setCmd(j,joint);
        }
    }
}

void State_FixedStand::exit(){
    _percent = 0;
}

FSMStateName State_FixedStand::checkChange(){
    if(_lowState->userCmd == UserCommand::L2_B){
        return FSMStateName::PASSIVE;
    }
    else if(_lowState->userCmd == UserCommand::L2_X){
        return FSMStateName::FREESTAND;
    }
    else if(_lowState->userCmd == UserCommand::START){
        return FSMStateName::TROTTING;
    }
    else if(_lowState->userCmd == UserCommand::L1_X){
        return FSMStateName::BALANCETEST;
    }
    else if(_lowState->userCmd == UserCommand::L1_A){
        return FSMStateName::SWINGTEST;
    }
    else if(_lowState->userCmd == UserCommand::L1_Y){
        return FSMStateName::STEPTEST;
    }
#ifdef COMPILE_WITH_MOVE_BASE
    else if(_lowState->userCmd == UserCommand::L2_Y){
        return FSMStateName::MOVE_BASE;
    }
#endif  // COMPILE_WITH_MOVE_BASE
    else if(_lowState->userCmd == UserCommand::RL){
        return FSMStateName::RL;
    }
    else{
        return FSMStateName::FIXEDSTAND;
    }
}
