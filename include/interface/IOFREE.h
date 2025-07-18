#ifndef IOFREE_H
#define IOFREE_H

#include <memory>
#include <string>
#include <vector>
#include "interface/IOInterface.h"
#include <fdsc_utils/free_dog_sdk_h.hpp>
#include "interface/KeyBoard.h"
class IOFREE : public IOInterface {
public:
    IOFREE();
    ~IOFREE() override = default;
    void sendRecv(const LowlevelCmd *cmd, LowlevelState *state) override;

private:
    std::shared_ptr<FDSC::UnitreeConnection> udp_;
    FDSC::lowCmd lowCmd_;
    FDSC::lowState lowState_;
    FDSC::MotorCmdArray mCmdArr_;
};

#endif  // IOFREE_H