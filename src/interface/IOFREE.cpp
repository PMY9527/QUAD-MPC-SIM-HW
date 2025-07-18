#ifdef COMPILE_WITH_REAL_ROBOT

#include "interface/IOFREE.h"
#include "interface/WirelessHandle.h"
#include <iostream>

IOFREE::IOFREE() {
    std::cout << "The control interface for real robot" << std::endl;
    cmdPanel = new KeyBoard();
    udp_ = std::make_shared<FDSC::UnitreeConnection>("LOW_WIRED_DEFAULTS");
    udp_->startRecv();
    std::vector<uint8_t> cmd_bytes = lowCmd_.buildCmd(false);
    udp_->send(cmd_bytes);
}

void IOFREE::sendRecv(const LowlevelCmd *cmd, LowlevelState *state) {
    // takes the lowlevelcmd from controller ----> mcmdarr (free_dog_sdk_cpp).
    state->userCmd = cmdPanel->getUserCmd();       
    state->userValue = cmdPanel->getUserValue();  
    for (int i = 0; i < 12; ++i) {
        mCmdArr_.motors[i].mode = FDSC::MotorModeLow::Servo;
        mCmdArr_.motors[i].q = static_cast<float>(cmd->motorCmd[i].q);
        mCmdArr_.motors[i].dq = static_cast<float>(cmd->motorCmd[i].dq);
        mCmdArr_.motors[i].Kp = static_cast<float>(cmd->motorCmd[i].Kp);
        mCmdArr_.motors[i].Kd = static_cast<float>(cmd->motorCmd[i].Kd);
        mCmdArr_.motors[i].tau = static_cast<float>(cmd->motorCmd[i].tau);
    }
    lowCmd_.motorCmd = mCmdArr_;

    // pack them up and send.
    std::vector<uint8_t> cmdBytes = lowCmd_.buildCmd(false);
    udp_->send(cmdBytes);

    // receive, decode and store sensor values to the lowlevelstate for the controller to read.
    std::vector<std::vector<uint8_t>> dataall;
    udp_->getData(dataall);
      if (dataall.size()!=0) {
        std::vector<uint8_t> data = dataall.at(dataall.size()-1);
        lowState_.parseData(data);

        for (int i = 0; i < 12; ++i) {
            state->motorState[i].q = lowState_.motorState[i].q;
            state->motorState[i].dq = lowState_.motorState[i].dq;
            //state->motorState[i].ddq = lowState_.motorState[i].ddq;
            state->motorState[i].tauEst = lowState_.motorState[i].tauEst;
            //state->motorState[i].mode = lowState_.motorState[i].mode;
        }
        for (int i = 0; i < 3; ++i) {
            state->imu.quaternion[i] = lowState_.imu_quaternion[i];
            state->imu.gyroscope[i] = lowState_.imu_gyroscope[i];
            state->imu.accelerometer[i] = lowState_.imu_accelerometer[i];
        }
        state->imu.quaternion[3] = lowState_.imu_quaternion[3];
    }
}

#endif  // COMPILE_WITH_REAL_ROBOT