
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


bool X7Controller::changeAngle(const uint8_t id, const double angle, const bool debug=false){
    // 指定IDのサーボを指定角度に動かす
    // 駆動に成功したらServoMapを更新する
    // debugのときはServoMapのみを更新する

    uint8_t dxl_error;
    int servoPosition = mServoMap[id].angleToPosition(angle);
    
    if(debug){
        mServoMap[id].setPosition(servoPosition);
        return false;
    }

    int result = mPacketHandler->write4ByteTxRx(
            mPortHandler,
            id,
            mADDR_GOAL_POSITION,
            servoPosition,
            &dxl_error);

    if(result != COMM_SUCCESS){
        std::cerr<<mPacketHandler->getTxRxResult(result)<<std::endl;
        return false;
    }else{
        mServoMap[id].setPosition(servoPosition);
        return true;
    }
}

bool X7Controller::changeVelocity(const uint8_t id, const double v_rad_per_sec, const bool debug=false){
    // 指定IDのサーボを指定角速度で動かす
    // velocityMode() で指定IDのサーボを速度制御にしておくこと
    //

    const double DXL_UNIT = 0.229; // RPM
    const double RPS_TO_DXL_RPM = 60.0 / (2.0 * M_PI * DXL_UNIT);

    uint8_t dxl_error;
    int32_t dxl_goal_velocity;

    dxl_goal_velocity = v_rad_per_sec * RPS_TO_DXL_RPM;
    
    std::cout<<"["<<std::to_string(id)<<"] "<<"goalVelocity: "<<dxl_goal_velocity<<std::endl;
    if(debug){
        return false;
    }

    int32_t result = mPacketHandler->write4ByteTxRx(
            mPortHandler,
            id,
            mADDR_GOAL_VELOCITY,
            dxl_goal_velocity
            &dxl_error);

    uint32_t result_velocity;
    result = mPacketHandler->read4ByteTxRx(
            mPortHandler,
            id,
            mADDR_GOAL_VELOCITY,
            (uint32_t*)&result_velocity,
            &dxl_error);
    std::cout<<"["<<std::to_string(id)<<"] "<<"result_velocity: "<<result_velocity<<std::endl;



    if(result != COMM_SUCCESS){
        std::cerr<<"error"<<mPacketHandler->getTxRxResult(result)<<std::endl;
        return false;
    }else if(dxl_error !=0){
        std::cerr<<"error"<<std::endl;
        return false;
    }
    return true;
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


bool X7Controller::velocityMode(const std::vector<uint8_t> &idList){
    // 指定IDのサーボを速度制御モードにする
    // この操作中はトルクがOFFされるため注意すること
    // 操作に失敗した場合にfalseを返す。ただし、失敗後も操作は続ける

    bool allComplete = true;
    int result;
    uint8_t dxl_error;
    std::vector<uint8_t> dummyList;

    // トルクをオフする
    torqueOnOff(dummyList, idList);

    // モードを切り替える
    for(uint8_t id : idList){
        result = mPacketHandler->write1ByteTxRx(
            mPortHandler,
            id,
            mADDR_OPERATING_MODE,
            mMODE_VELOCITY_CONTROL,
            &dxl_error);
    
        if(result != COMM_SUCCESS){
            std::cerr<<mPacketHandler->getTxRxResult(result)<<std::endl;
            allComplete = false;
        }
    }

    // トルクをオンする
    torqueOnOff(idList, dummyList);

    return allComplete;
}


bool X7Controller::positionMode(const std::vector<uint8_t> &idList){
    // 指定IDのサーボを位置制御モードにする
    // この操作中はトルクがOFFされるため注意すること
    // 操作に失敗した場合にfalseを返す。ただし、失敗後も操作は続ける

    bool allComplete = true;
    int result;
    uint8_t dxl_error;
    std::vector<uint8_t> dummyList;

    // トルクをオフする
    torqueOnOff(dummyList, idList);

    // モードを切り替える
    for(uint8_t id : idList){
        result = mPacketHandler->write1ByteTxRx(
            mPortHandler,
            id,
            mADDR_OPERATING_MODE,
            mMODE_POSITION_CONTROL,
            &dxl_error);
    
        if(result != COMM_SUCCESS){
            std::cerr<<mPacketHandler->getTxRxResult(result)<<std::endl;
            allComplete = false;
        }
    }

    // トルクをオンする
    torqueOnOff(idList, dummyList);

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

    if(mPositionInitialized){
        changeAngle(2, angle2, debug);
        changeAngle(3, angle3, debug);
        changeAngle(5, angle5, debug);
    }else{
        std::cout<<"Please initialize servo position"<<std::endl;
        return false;
    }

    return true;
}

bool X7Controller::move23578(const double x, const double y, const double z, const double beta, const double gamma, const bool debug){

    // ID2とID3とID5のサーボを駆動し、指定座標(x,y,z)へハンドを動かす
    // ハンドをy軸回りにbetaだけ回転させた姿勢に動かす
    // 指定座標が駆動範囲外の場合falseを返す


    if(move2_3_5(x, y, z, debug) == false){
        return false;
    }

    double angle2 = mServoMap[2].getAngle();
    double angle3 = mServoMap[3].getAngle();
    double angle5 = mServoMap[5].getAngle();

    // ID7と8のサーボの駆動範囲を-π/2 ~ +π/2に制限する
    double angle7 = beta - (angle3 + angle5);
    double angle8 = gamma + angle2;


    if(angle7 > M_PI_2){
        std::cerr<<"betaがプラス方向に大きい "
            <<"beta: "<<std::to_string(beta)
            <<" angle7: "<<std::to_string(angle7)<<std::endl;
        return false;

    }else if(angle7 < -M_PI_2){
        std::cerr<<"betaがマイナス方向に大きい "
            <<"beta: "<<std::to_string(beta)
            <<" angle7: "<<std::to_string(angle7)<<std::endl;
        return false;
    }

    std::cout<<"Angle7: "<<std::to_string(toDegree(angle7))<<std::endl;
    changeAngle(7, angle7, debug);
    changeAngle(8, angle8, debug);

    return true;
}


bool X7Controller::testMove(const int duration_msec){
    // 指定時間、テスト動作をする
    std::chrono::system_clock::time_point  start, end;
    start = std::chrono::system_clock::now();
    double elapsed = 0;

    if(mPositionInitialized == false){
        std::cout<<"Please initialize servo position"<<std::endl;
        return false;
    }

    // ID3と5だけトルクをオフする
    std::vector<uint8_t> dummyList;
    std::vector<uint8_t> idList = {3, 5};
    torqueOnOff(dummyList, idList);

    Complex position;
    bool velocityControl = false;

    while(elapsed < duration_msec){

        position = getPositionXZ();
        std::cout<<"X:"<<position.real()<<" Z:"<<position.imag()<<std::endl;
        if(position.imag() < 0.4){
            velocityControl = true;
            std::cout<<"VelocityControl"<<std::endl;
            break;
        }

        // 経過時間を更新
        end = std::chrono::system_clock::now();
        elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();
    }

    if(velocityControl){
        // 速度制御モードに切り替える
        velocityMode(idList);

        velocityMove(0.1, 0.0, 2000, false);

        // 位置制御モードに切り替える
        positionMode(idList);
    }

    // ID3と5のトルクをオンする
    torqueOnOff(idList, dummyList);

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


Complex X7Controller::getPositionXZ(void){
    // ID3とID5のサーボ位置を取得し、x, y座標として返す
    // 他のサーボはinitialize位置で固定されていること


    double angle3 = updateServoAngle(3);
    double angle5 = updateServoAngle(5);
    Complex position;

    // 実数部をx座標とする
    position.real(mLINK3*std::sin(angle3) + mLINK5*std::sin(angle3+angle5));
    position.imag(mLINK0 + mLINK3*std::cos(angle3) + mLINK5*std::cos(angle3+angle5));

    return position;
}


bool X7Controller::velocityMove(const double vx, const double vz, const int duration_msec, const bool debug){

    std::chrono::system_clock::time_point  start, end;
    start = std::chrono::system_clock::now();
    double elapsed = 0;

    while(elapsed < duration_msec){

        double angle3 = updateServoAngle(3);
        double angle5 = updateServoAngle(5);

        double A = -1.0 / (mLINK3 * mLINK5 * std::sin(angle5));
        double UP_LEFT = -mLINK5 * std::sin(angle3 + angle5);
        double UP_RIGHT= -mLINK5 * std::cos(angle3 + angle5);
        double DN_LEFT = mLINK3 * std::sin(angle3) + mLINK5 * std::sin(angle3 + angle5);
        double DN_RIGHT= mLINK3 * std::cos(angle3) + mLINK5 * std::cos(angle3 + angle5);

        double servoVel3 = A * (UP_LEFT*vx + UP_RIGHT*vz);
        double servoVel5 = A * (DN_LEFT*vx + DN_RIGHT*vz);

        std::cerr<<"ID3_VEL:"<<toDegree(servoVel3)
            <<" ID5_VEL:"<<toDegree(servoVel5)<<std::endl;

        // changeVelocity(3, servoVel3, debug);
        // changeVelocity(5, servoVel5, debug);
        changeVelocity(5, 0.7854, false);

        // 経過時間を更新
        end = std::chrono::system_clock::now();
        elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();
    }

    return true;
}


double X7Controller::updateServoAngle(const uint8_t id){
     uint32_t dxl_present_position;
     uint8_t dxl_error;

     int result = mPacketHandler->read4ByteTxRx(mPortHandler,id,
             mADDR_PRESENT_POSITION,(uint32_t*)&dxl_present_position,&dxl_error);
     mServoMap[id].setPosition(dxl_present_position);
     double angle = mServoMap[id].getAngle();

     return angle;
}
