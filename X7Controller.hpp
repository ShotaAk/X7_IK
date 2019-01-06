#ifndef   X7CONTROLLER_HPP
#define   X7CONTROLLER_HPP

#include <unordered_map>
#include <dynamixel_sdk.h>
#include <cmath>
#include <vector>

namespace dxl = dynamixel;

class Servo{
    public:
        Servo()
            : mMAX_POSITION(0), mMIN_POSITION(0), mOFFSET(0), mINVERT(1){
        }
        Servo(const int maxPos, const int minPos, const int offset, const double invert)
            : mMAX_POSITION(maxPos), mMIN_POSITION(minPos), mOFFSET(offset), mINVERT(invert){
        }
        ~Servo(){}

        void setPosition(const int pos){
            mCurrentPosition = pos;
        }
        int getPosition(void) const {
            return mCurrentPosition;
        }
        double getAngle(void) const {
            return mINVERT * 2.0 * M_PI * (double)(mCurrentPosition - mOFFSET)/(double)mRESOLUTION;
        }
        int angleToPosition(const double angle_radian) const {
            return mINVERT * (angle_radian / (2.0*M_PI)) * mRESOLUTION + mOFFSET;
        }

    private:
        const int mRESOLUTION = 4096;
        const int mMAX_POSITION;
        const int mMIN_POSITION;
        const int mOFFSET;
        const double mINVERT;

        int mCurrentPosition;
};

class X7Controller{
    public:
        X7Controller(const char *deviceName);
        ~X7Controller();

        bool initializePosition(void);
        void showServoAngles(const int duration_msec);
        bool changeAngle(const uint8_t id, const double angle, const bool debug);
        bool torqueOnOff(const std::vector<uint8_t> &onList, const std::vector<uint8_t> &offList);
        bool move3_5(const double x, const double z, const bool debug);
        bool move2_3_5(const double x, const double y, const double z, const bool debug);
        bool move23578(const double x, const double y, const double z, const double beta, const double gamma, const bool debug);

    private:
        void communicationCheck(void);
        void detachDynamixel(void);
        void initializeDxlParameters(void);

        
        const double    mLINK0                  = 0.11; // meter 固定リンク
        const double    mLINK3                  = 0.25; // meter 駆動リンク
        const double    mLINK5                  = 0.30; // meter 駆動リンク

        const double    mPROTOCOL_VERSION       = 2.0;
        const int       mBAUDRATE               = 3000000;
        const int       mPID_P_GAIN             = 400;
        const int       mTORQUE_ENABLE          = 1;
        const int       mTORQUE_DISABLE         = 0;
        const uint16_t  mADDR_TORQUE_ENABLE     = 64;
        const uint16_t  mADDR_POSITION_P_GAIN   = 84;
        const uint16_t  mADDR_GOAL_POSITION     = 116;
        const uint16_t  mADDR_PRESENT_POSITION  = 132;

        const std::vector<uint8_t> mID_LIST = {2,3,4,5,6,7,8,9};
        std::unordered_map<uint8_t, Servo> mServoMap = {
            {2, Servo(3834,262,2045,  1.0)},
            {3, Servo(3072,1024,2059,-1.0)},
            {4, Servo(3834,262,2057,  1.0)},
            {5, Servo(2048,228,2040, -1.0)},
            {6, Servo(3834,262,2040,  1.0)},
            {7, Servo(3072,1024,1994,-1.0)},
            {8, Servo(3948,148,2017,  1.0)},
            {9, Servo(3000,1991,2022, 1.0)},
        };

        dxl::PortHandler *mPortHandler;
        dxl::PacketHandler *mPacketHandler;
        bool mPositionInitialized;
};


#endif // X7CONTROLLER_HPP
