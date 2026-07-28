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
    // Headless Gazebo 的实际控制周期可能远慢于配置的 dt。若仍按 1000 个周期
    // 插值，解除物理暂停时机器人会在站姿尚未到位前跌倒；仿真入口直接给标准站姿。
    const char *headlessMode = std::getenv("SIMENV_HEADLESS_MODE");
    _percent = (headlessMode != nullptr && std::string(headlessMode) == "move_base")
        ? 1.0f : 0.0f;
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
