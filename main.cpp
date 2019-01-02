

#include "X7Controller.hpp"

#include <fcntl.h>
#include <termios.h>


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
            controller.changeAngle(3, M_PI*45.0/180.0);
            controller.changeAngle(5, M_PI*45.0/180.0);
        }

    }while(finish == false);

    return 0;

}
