
#include "X7Controller.hpp"

#include <iostream>
#include <cmath>
#include <cstdlib>
#include <vector>
#include <chrono>



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


X7Controller::X7Controller(const char *deviceName){
    mPortHandler = dxl::PortHandler::getPortHandler(deviceName);
    mPacketHandler = dxl::PacketHandler::getPacketHandler(mPROTOCOL_VERSION);


    // Port Open
    if(mPortHandler->openPort() == false){
        std::cerr<<"Failed to open the port:" << deviceName << std::endl;
        std::exit(1);
    }

    // Set Baudrate
    if(mPortHandler->setBaudRate(mBAUDRATE) == false){
        std::cerr<<"Failed to change the baudrate:" << mBAUDRATE << std::endl;
        detachDynamixel();
        std::exit(1);
    }

    communicationCheck();

    initializeDxlParameters();

    mServoMap->emplace(2, Servo(3834,262,2045,  1.0));
    mServoMap->emplace(3, Servo(3072,1024,2059,-1.0));
    mServoMap->emplace(4, Servo(3834,262,2057,  1.0));
    mServoMap->emplace(5, Servo(2048,228,2040, -1.0));
    mServoMap->emplace(6, Servo(3834,262,2040,  1.0));
    mServoMap->emplace(7, Servo(3072,1024,1994, 1.0));
    mServoMap->emplace(8, Servo(3948,148,2017,  1.0));
    mServoMap->emplace(9, Servo(3000,1991,2022, 1.0));
}

X7Controller::~X7Controller(){
    detachDynamixel();
}


bool X7Controller::initializePosition(void){
    int result;
    uint8_t dxl_error;
    for(uint8_t id : mID_LIST){
        result = mPacketHandler->write4ByteTxRx(
            mPortHandler,
            id,
            mADDR_GOAL_POSITION,
            mServoMap->at(id).angleToPosition(0),
            &dxl_error);
    }
}


void X7Controller::showServoAngles(const int duration_msec){
    // 指定時間、各サーボの角度を表示する
     std::chrono::system_clock::time_point  start, end;
     start = std::chrono::system_clock::now();
     double elapsed = 0;

     uint32_t dxl_present_position;
     uint8_t dxl_error;

     while(elapsed < duration_msec){
         std::cout<<"Servo Angles: \t";
         for(uint8_t id : mID_LIST){
             int result = mPacketHandler->read4ByteTxRx(
                     mPortHandler,
                     id,
                     mADDR_PRESENT_POSITION,
                     (uint32_t*)&dxl_present_position,
                     &dxl_error);
             
             if(result == COMM_SUCCESS){
                 mServoMap->at(id).setPosition(dxl_present_position);
                 std::cout<<"["<<std::to_string(id)<<"]"<<
                     std::to_string(mServoMap->at(id).getAngle())<<"\t";
             }
         }
         std::cout<<std::endl;

         // 経過時間を更新
         end = std::chrono::system_clock::now();
         elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();
     }
}


bool X7Controller::changeAngle(const uint8_t id, const double angle){

    uint8_t dxl_error;
    int result = mPacketHandler->write4ByteTxRx(
            mPortHandler,
            id,
            mADDR_GOAL_POSITION,
            mServoMap->at(id).angleToPosition(angle),
            &dxl_error);

    if(result != COMM_SUCCESS){
        std::cerr<<mPacketHandler->getTxRxResult(result)<<std::endl;
        return false;
    }else{
        return true;
    }
}

void X7Controller::communicationCheck(void){
    // Try to broadcast ping the Dynamixel
    std::vector<uint8_t> vec;
    int result = mPacketHandler->broadcastPing(mPortHandler, vec);
    if (result != COMM_SUCCESS){
        std::cerr<< mPacketHandler->getTxRxResult(result)<<std::endl;
        detachDynamixel();
        std::exit(1);
    }else{
        std::cout<<"Detected Dynamixel :";
        for(int id : vec){
            std::cout<<"[ID:"<<id<<"], ";
        }
        std::cout<<std::endl;

        if(vec == mID_LIST){
            std::cout<<"ID Complete"<<std::endl;
        }else{
            std::cout<<"There are missing ID"<<std::endl;
            detachDynamixel();
            std::exit(1);
        }
    }
}


void X7Controller::detachDynamixel(void){
    // トルク解除とポートの切断をする
    int result;
    uint8_t dxl_error;
    for(uint8_t id : mID_LIST){
        result = mPacketHandler->write1ByteTxRx(
            mPortHandler,
            id,
            mADDR_TORQUE_ENABLE,
            mTORQUE_DISABLE,
            &dxl_error);
    }

    std::cout<<"Close the port"<<std::endl;
    mPortHandler->closePort();
}


void X7Controller::initializeDxlParameters(void){
    // PIDゲインの設定後トルクをオンする
    int result;
    uint8_t dxl_error;
    for(uint8_t id : mID_LIST){
        result = mPacketHandler->write2ByteTxRx(
            mPortHandler,
            id,
            mADDR_POSITION_P_GAIN,
            mPID_P_GAIN,
            &dxl_error);

        result = mPacketHandler->write1ByteTxRx(
            mPortHandler,
            id,
            mADDR_TORQUE_ENABLE,
            mTORQUE_ENABLE,
            &dxl_error);
    }
}
