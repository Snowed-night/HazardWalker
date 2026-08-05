/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "FSM/FSM.h"
#include <algorithm>
#include <cstdlib>
#include <iostream>

FSM::FSM(CtrlComponents *ctrlComp)
    :_ctrlComp(ctrlComp),
     _stateEnteredTime(0),
     _headlessEnabled(false),
     _headlessAutoRl(false),
     _headlessStandDelaySec(5.0),
     _headlessRlDelaySec(2.0){

    _stateList.invalid = nullptr;
    _stateList.passive = new State_Passive(_ctrlComp);
    _stateList.fixedStand = new State_FixedStand(_ctrlComp);
    _stateList.freeStand = new State_FreeStand(_ctrlComp);
    _stateList.trotting = new State_Trotting(_ctrlComp);
    _stateList.balanceTest = new State_BalanceTest(_ctrlComp);
    _stateList.swingTest = new State_SwingTest(_ctrlComp);
    _stateList.stepTest = new State_StepTest(_ctrlComp);
#ifdef COMPILE_WITH_MOVE_BASE
    _stateList.moveBase = new State_move_base(_ctrlComp);
#endif  // COMPILE_WITH_MOVE_BASE
    _stateList.rl = new State_RL(_ctrlComp);
    initialize();
}

FSM::~FSM(){
    _stateList.deletePtr();
}

void FSM::initialize(){
    _currentState = _stateList.passive;
    _currentState -> enter();
    _nextState = _currentState;
    _mode = FSMMode::NORMAL;
    _stateEnteredTime = getSystemTime();

    const char *headlessMode = std::getenv("SIMENV_HEADLESS_MODE");
    _headlessMode = headlessMode == nullptr ? "" : headlessMode;
    _headlessEnabled = (_headlessMode == "move_base");
    const char *autoRl = std::getenv("SIMENV_AUTO_RL");
    _headlessAutoRl = autoRl != nullptr && std::string(autoRl) == "1";
    const char *standDelay = std::getenv("CONTROLLER_AUTO_STAND_DELAY_SEC");
    const char *rlDelay = std::getenv("CONTROLLER_AUTO_RL_DELAY_SEC");
    if (standDelay != nullptr) {
        _headlessStandDelaySec = std::max(0.0, std::atof(standDelay));
    }
    if (rlDelay != nullptr) {
        _headlessRlDelaySec = std::max(0.0, std::atof(rlDelay));
    }
    if (_headlessEnabled) {
        std::cout << "[HEADLESS_FSM] mode=" << _headlessMode
                  << " auto_rl=" << (_headlessAutoRl ? 1 : 0) << std::endl;
    }
}

void FSM::run(){
    _startTime = getSystemTime();
    _ctrlComp->sendRecv();
    _ctrlComp->ioInterFreeDog->sendRecv();
    _ctrlComp->runWaveGen();
    _ctrlComp->estimator->run();
    if(!checkSafty()){
        // _ctrlComp->ioInter->setPassive();
    }

    if(_mode == FSMMode::NORMAL){
        _currentState->run();
        _nextStateName = _headlessEnabled
            ? getHeadlessNextState()
            : _currentState->checkChange();
        if(_nextStateName != _currentState->_stateName){
            _mode = FSMMode::CHANGE;
            _nextState = getNextState(_nextStateName);
            std::cout << "Switched from " << _currentState->_stateNameString
                      << " to " << _nextState->_stateNameString << std::endl;
        }
    }
    else if(_mode == FSMMode::CHANGE){
        _currentState->exit();
        _currentState = _nextState;
        _currentState->enter();
        _stateEnteredTime = getSystemTime();
        _mode = FSMMode::NORMAL;
        _currentState->run();
    }

    absoluteWait(_startTime, (long long)(_ctrlComp->dt * 1000000));
}

FSMState* FSM::getNextState(FSMStateName stateName){
    switch (stateName)
    {
    case FSMStateName::INVALID:
        return _stateList.invalid;
        break;
    case FSMStateName::PASSIVE:
        return _stateList.passive;
        break;
    case FSMStateName::FIXEDSTAND:
        return _stateList.fixedStand;
        break;
    case FSMStateName::FREESTAND:
        return _stateList.freeStand;
        break;
    case FSMStateName::TROTTING:
        return _stateList.trotting;
        break;
    case FSMStateName::BALANCETEST:
        return _stateList.balanceTest;
        break;
    case FSMStateName::SWINGTEST:
        return _stateList.swingTest;
        break;
    case FSMStateName::STEPTEST:
        return _stateList.stepTest;
        break;
#ifdef COMPILE_WITH_MOVE_BASE
    case FSMStateName::MOVE_BASE:
        return _stateList.moveBase;
        break;
#endif  // COMPILE_WITH_MOVE_BASE
    case FSMStateName::RL:
        return _stateList.rl;
    break;
    default:
        return _stateList.invalid;
        break;
    }
}

bool FSM::checkSafty(){
    // The angle with z axis less than 60 degree
    if(_ctrlComp->lowState->getRotMat()(2,2) < 0.5 ){
        return false;
    }else{
        return true;
    }
}

FSMStateName FSM::getHeadlessNextState(){
    const double stateAgeSec =
        static_cast<double>(getSystemTime() - _stateEnteredTime) / 1000000.0;
    if (_currentState->_stateName == FSMStateName::PASSIVE) {
        // 物理解除暂停后的首个周期立即接管为站立，避免在无力矩状态等待时跌倒。
        return FSMStateName::FIXEDSTAND;
    }
    if (_currentState->_stateName == FSMStateName::FIXEDSTAND) {
        // 无速度命令时始终固定站立；只有用户实际发出非零 /cmd_vel 才进入
        // 行走控制器。SIMENV_AUTO_RL=1 选择学习策略，=0 选择官方经典
        // move_base/步态控制器，便于在低实时倍率或策略不稳定时安全回退。
        if (!_stateList.fixedStand->hasFreshMotionCommand()) {
            return FSMStateName::FIXEDSTAND;
        }
        if (_headlessAutoRl) {
            return FSMStateName::RL;
        }
#ifdef COMPILE_WITH_MOVE_BASE
        return FSMStateName::MOVE_BASE;
#else
        return FSMStateName::FIXEDSTAND;
#endif
    }
    if (_currentState->_stateName == FSMStateName::RL
            || _currentState->_stateName == FSMStateName::MOVE_BASE) {
        // 已切入行走状态后保持该状态。各控制器自行处理速度更新；不能再读取
        // fixedStand 对象的时间戳，否则正常行走会被误切回固定站立。
        return _currentState->_stateName;
    }
    // 其他状态保持不变。
    return _currentState->_stateName;
}
