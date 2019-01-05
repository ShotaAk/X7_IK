
#include "X7Controller.hpp"

#include <iostream>
#include <cstdlib>
#include <chrono>


inline double toDegree(double x){return 180.0 * x / M_PI;}
inline double toRadian(double x){return M_PI * x / 180.0;}




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
    mPositionInitialized = false;
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
            mServoMap[id].angleToPosition(0),
            &dxl_error);
    }

    mPositionInitialized = true;

    return true;
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
                 mServoMap[id].setPosition(dxl_present_position);
                 std::cout<<"["<<std::to_string(id)<<"]"<<
                     std::to_string(mServoMap[id].getAngle())<<"\t";
             }
         }
         std::cout<<std::endl;

         // 経過時間を更新
         end = std::chrono::system_clock::now();
         elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();
     }
}


bool X7Controller::changeAngle(const uint8_t id, const double angle){
    // 指定IDのサーボを指定角度に動かす

    uint8_t dxl_error;
    int result = mPacketHandler->write4ByteTxRx(
            mPortHandler,
            id,
            mADDR_GOAL_POSITION,
            mServoMap[id].angleToPosition(angle),
            &dxl_error);

    if(result != COMM_SUCCESS){
        std::cerr<<mPacketHandler->getTxRxResult(result)<<std::endl;
        return false;
    }else{
        return true;
    }
}

bool X7Controller::torqueOnOff(const std::vector<uint8_t> &onList, const std::vector<uint8_t> &offList){
    // 指定IDのサーボのトルクをON/OFFする
    // トルク操作に失敗した場合にfalseを返す。ただし、失敗後も操作は続ける
    // onListとoffListに同じIDが含まれていてもOK。トルクオフ動作が優先される

    bool allComplete = true;
    int result;
    uint8_t dxl_error;

    // トルクオン
    for(uint8_t id : onList){
        result = mPacketHandler->write1ByteTxRx(
            mPortHandler,
            id,
            mADDR_TORQUE_ENABLE,
            mTORQUE_ENABLE,
            &dxl_error);

        if(result != COMM_SUCCESS){
            std::cerr<<mPacketHandler->getTxRxResult(result)<<std::endl;
            allComplete = false;
        }
    }

    // トルクオフ
    for(uint8_t id : offList){
        result = mPacketHandler->write1ByteTxRx(
            mPortHandler,
            id,
            mADDR_TORQUE_ENABLE,
            mTORQUE_DISABLE,
            &dxl_error);

        if(result != COMM_SUCCESS){
            std::cerr<<mPacketHandler->getTxRxResult(result)<<std::endl;
            allComplete = false;
        }
    }

    return allComplete;
}


bool X7Controller::move3_5(const double x, const double z, const bool debug = true){
    // ID3とID5のサーボを駆動し、指定座標(x, z)へハンドを動かす
    // 指定座標が駆動範囲外の場合falseを返す

    double z_0 = z - mLINK0; // 地面から浮いてる長さを引く
    double inArcCos5 = (std::pow(x,2) 
            + std::pow(z_0, 2) 
            - std::pow(mLINK3, 2)
            - std::pow(mLINK5, 2)) / (2.0*mLINK3*mLINK5);

    if(inArcCos5 < -1.0 || inArcCos5 > 1.0){
        std::cerr<<"角度計算がダメ angle5"<<std::endl;
        return false;
    }
    double angle5 = std::acos(inArcCos5);

    double A = mLINK3 + mLINK5*std::cos(angle5); // 計算用の変数
    double B = mLINK5*std::sin(angle5); // 計算用の変数
    double inArcSin3 = (A*x - B*z_0)/(std::pow(A,2) + std::pow(B,2));

    if(inArcSin3 < -1.0 || inArcSin3 > 1.0){
        std::cerr<<"角度計算がダメ angle3"<<std::endl;
        return false;
    }
    double angle3 = std::asin(inArcSin3);

    std::cout<<"Angle3 :"<<std::to_string(toDegree(angle3))
        <<", Angle5 :"<<std::to_string(toDegree(angle5))<<std::endl;

    if(debug) return true;

    if(mPositionInitialized){
        changeAngle(3, angle3);
        changeAngle(5, angle5);
    }else{
        std::cout<<"Please initialize servo position"<<std::endl;
        return false;
    }

    return true;
}


bool X7Controller::move2_3_5(const double x, const double y, const double z, const bool debug){
    // ID2とID3とID5のサーボを駆動し、指定座標(x,y,z)へハンドを動かす
    // 指定座標が駆動範囲外の場合falseを返す

    double z_0 = z - mLINK0; // 地面から浮いてる長さを引く
    double inArcCos5 = (std::pow(x,2) 
            + std::pow(y, 2) 
            + std::pow(z_0, 2) 
            - std::pow(mLINK3, 2)
            - std::pow(mLINK5, 2)) / (2.0*mLINK3*mLINK5);

    if(inArcCos5 < -1.0 || inArcCos5 > 1.0){
        std::cerr<<"角度計算がダメ angle5"<<std::endl;
        return false;
    }
    double angle5 = std::acos(inArcCos5);

    double angle2 = std::atan(y/x);

    // 計算用の変数
    double C2 = std::cos(angle2);
    double C5 = std::cos(angle5);
    double S5 = std::sin(angle5);

    double A = mLINK3*C2 + mLINK5*C2*C5;
    double B = mLINK5*C2*S5;
    double D = mLINK3 + mLINK5*C5;
    double E = mLINK5*S5;

    double inArcSin3 = (D*x - B*z_0)/(A*D + B*E);

    if(inArcSin3 < -1.0 || inArcSin3 > 1.0){
        std::cerr<<"角度計算がダメ angle3"<<std::endl;
        return false;
    }
    double angle3 = std::asin(inArcSin3);

    std::cout
        <<"Angle2 :"<<std::to_string(toDegree(angle2))
        <<", Angle3 :"<<std::to_string(toDegree(angle3))
        <<", Angle5 :"<<std::to_string(toDegree(angle5))<<std::endl;

    if(debug) return true;

    if(mPositionInitialized){
        changeAngle(2, angle2);
        changeAngle(3, angle3);
        changeAngle(5, angle5);
    }else{
        std::cout<<"Please initialize servo position"<<std::endl;
        return false;
    }

    return true;
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
