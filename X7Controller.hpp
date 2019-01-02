#ifndef   X7CONTROLLER_HPP
#define   X7CONTROLLER_HPP

#include <unordered_map>
#include <dynamixel_sdk.h>

namespace dxl = dynamixel;

class Servo;

class X7Controller{
    public:
        X7Controller(const char *deviceName);
        ~X7Controller();

        bool initializePosition(void);
        void showServoAngles(const int duration_msec);
        bool changeAngle(const uint8_t id, const double angle);


    private:
        void communicationCheck(void);
        void detachDynamixel(void);
        void initializeDxlParameters(void);


        const double    mPROTOCOL_VERSION       = 2.0;
        const int       mBAUDRATE               = 3000000;
        const int       mPID_P_GAIN             = 300;
        const int       mTORQUE_ENABLE          = 1;
        const int       mTORQUE_DISABLE         = 0;
        const uint16_t  mADDR_TORQUE_ENABLE     = 64;
        const uint16_t  mADDR_POSITION_P_GAIN   = 84;
        const uint16_t  mADDR_GOAL_POSITION     = 116;
        const uint16_t  mADDR_PRESENT_POSITION  = 132;

        const std::vector<uint8_t> mID_LIST = {2,3,4,5,6,7,8,9};
        std::unordered_map<uint8_t, Servo> *mServoMap;

        dxl::PortHandler *mPortHandler;
        dxl::PacketHandler *mPacketHandler;
};


#endif // X7CONTROLLER_HPP
