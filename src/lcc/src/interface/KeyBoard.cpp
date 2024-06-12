/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "interface/KeyBoard.h"
// #include "interface/ "
#include <iostream>

KeyBoard::KeyBoard(){
    userCmd = UserCommand::NONE;
    userValue.setZero();

    tcgetattr( fileno( stdin ), &_oldSettings );
    _newSettings = _oldSettings;
    _newSettings.c_lflag &= (~ICANON & ~ECHO);
    tcsetattr( fileno( stdin ), TCSANOW, &_newSettings );

    pthread_create(&_tid, NULL, runKeyBoard, (void*)this);
}

KeyBoard::~KeyBoard(){
    pthread_cancel(_tid);
    pthread_join(_tid, NULL);
    tcsetattr( fileno( stdin ), TCSANOW, &_oldSettings );
}

UserCommand KeyBoard::checkCmd(){
    switch (_c){
    case '1':
        return UserCommand::PASSIVE_1;
    case '2':
        return UserCommand::FIXEDSTAND_2;
    case '3':
        return UserCommand::FREESTAND_3;
    case '4':
        // printf(" \n keyboard->VMC_4  \n ");
        return UserCommand::VMC_4;
    case '5':
        return UserCommand::TROTTING_5;

#ifdef COMPILE_WITH_MOVE_BASE
    case '5':
        return UserCommand::L2_Y;
#endif  // COMPILE_WITH_MOVE_BASE

    case '6':
        return UserCommand::A1MPC_6;

    case '0':
        return UserCommand::BALANCE_TEST0;
    case '9':
        return UserCommand::SWING_TEST9;
    case '8':
        return UserCommand::SETP_TEST8;
    case ' ':
        userValue.setZero();
        return UserCommand::NONE;
    default:
        return UserCommand::NONE;
    }
}

void KeyBoard::changeValue(){
    switch (_c){
    case 'w':case 'W':
        userValue.ly = min<float>(userValue.ly+sensitivityLeft, 1.0);
        break;
    case 's':case 'S':
        userValue.ly = max<float>(userValue.ly-sensitivityLeft, -1.0);
        break;
    case 'd':case 'D':
        userValue.lx = min<float>(userValue.lx+sensitivityLeft, 1.0);
        break;
    case 'a':case 'A':
        userValue.lx = max<float>(userValue.lx-sensitivityLeft, -1.0);
        break;

    case 'i':case 'I':
        userValue.ry = min<float>(userValue.ry+sensitivityRight, 1.0);
        break;
    case 'k':case 'K':
        userValue.ry = max<float>(userValue.ry-sensitivityRight, -1.0);
        break;
    case 'l':case 'L':
        userValue.rx = min<float>(userValue.rx+sensitivityRight, 1.0);
        break;
    case 'j':case 'J':
        userValue.rx = max<float>(userValue.rx-sensitivityRight, -1.0);
        break;
    default:
        break;
    }
}

// lcc 20250601
void KeyBoard::changeFunctionModeValue(){
    switch (_c){
    case 't':case 'T':
        {
            if( userFunctionMode.function_test == false )
                userFunctionMode.function_test = true;
            else if( userFunctionMode.function_test == true )
                userFunctionMode.function_test = false;
            std::cout<<" function_test:  "<< userFunctionMode.function_test <<std::endl;
        }
        break;
    default:
        break;
    }
}


void* KeyBoard::runKeyBoard(void *arg){
    ((KeyBoard*)arg)->run(NULL);
    return NULL;
}

void* KeyBoard::run(void *arg){
    while(1){
        FD_ZERO(&set);
        FD_SET( fileno( stdin ), &set );

        res = select( fileno( stdin )+1, &set, NULL, NULL, NULL);

        if(res > 0){
            ret = read( fileno( stdin ), &_c, 1 );
            userCmd = checkCmd();
            if(userCmd == UserCommand::NONE)
                changeValue();
                changeFunctionModeValue();// lcc 20250601
            _c = '\0';
        }
        usleep(1000);
    }
    return NULL;
}