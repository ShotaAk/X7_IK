
#include "X7Controller.hpp"

#include <fcntl.h>
#include <termios.h>
#include <iostream>
#include <cmath>
#include <unistd.h>


int getch(){
    struct termios oldt, newt;
    int ch;
    tcgetattr(0, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(0, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(0, TCSANOW, &oldt);
    return ch;
}

void demo35Circle(X7Controller &controller, const bool debug){
    // 円運動
    std::cout<<"円運動"<<std::endl;
    int step_max = 60;
    int rotation_num = 2;
    double offset_x = 0.1, offset_z = 0.4;
    double length = 0.10;
    double theta = 0;
    double x, z;

    // 最初に初期位置へ動かす
    x = offset_x + length*cos(theta);
    z = offset_z + length*sin(theta);
    if(controller.move3_5(x, z, debug) == false){
        return;
    }
    usleep(2e6);

    for(int r_num=0; r_num<rotation_num; r_num++){
        for(int i=0; i<step_max; i++){
            theta = 2.0*M_PI* (i/(double)step_max);

            x = offset_x + length*cos(theta);
            z = offset_z + length*sin(theta);

            std::cout<<"x:z"<<std::to_string(x)<<":"<<std::to_string(z)<<std::endl;
            if(controller.move3_5(x, z, debug) == false){
                return;
            }
            usleep(1e5);
        }
    }
}

void demo235Circle(X7Controller &controller, const bool debug){
    // 円運動
    std::cout<<"円運動"<<std::endl;
    int step_max = 30;
    int rotation_num = 2;
    double offset_x = 0.2, offset_y = 0.0, offset_z = 0.4;
    double length = 0.1;
    double theta = 0;
    double x, y, z;

    // 最初に初期位置へ動かす
    x = offset_x;
    y = offset_y + length*cos(theta);
    z = offset_z + length*sin(theta);
    if(controller.move2_3_5(x, y, z, debug) == false){
        return;
    }
    usleep(2e6);

    for(int r_num=0; r_num<rotation_num; r_num++){
        for(int i=0; i<step_max; i++){
            theta = 2.0*M_PI* (i/(double)step_max);

            y = offset_y + length*cos(theta);
            z = offset_z + length*sin(theta);

            std::cout<<"x:y:z"
                <<std::to_string(x)<<":"
                <<std::to_string(y)<<":"
                <<std::to_string(z)<<std::endl;
            if(controller.move2_3_5(x, y, z, debug) == false){
                return;
            }
            usleep(1e5);
        }
    }
}


int main(int argc, char* argv[]){

    if(argc < 2){
        std::cerr<<"引数にDeviceName (/dev/ttyUSB0) を入力してね" << std::endl;
        std::exit(1);
    }
    X7Controller controller(argv[1]);


    bool finish = false;

    do{
        std::cout<< "Please enter any key (q: quit)"<<std::endl;
        char keyInput = getch();
        if(keyInput == 'q'){
            finish = true;
        }else if(keyInput == 's'){
            controller.showServoAngles(10000);
        }else if(keyInput == 'i'){
            controller.initializePosition();
        }else if(keyInput == 'a'){
            // demo35Circle(controller, true);
            // demo235Circle(controller, false);
            std::vector<uint8_t> onList = {4};
            std::vector<uint8_t> offList = {2, 3, 5, 6, 7, 8 ,9};
            controller.torqueOnOff(onList, offList);
        }else{
            std::cout<<keyInput<<std::endl;
        }

    }while(finish == false);

    return 0;

}
