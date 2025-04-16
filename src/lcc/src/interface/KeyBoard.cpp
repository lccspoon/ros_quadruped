#include "interface/KeyBoard.h"
// #include "interface/ "
#include <iostream>
#include "interface/IOSDK.h"

bool KEY_M = false;
bool USVLCC_SETZERO = false;
bool FORCE_PROTECT_CHANGE = false;
bool DTAT_SAVE2TXT = false;

std::mutex MTX_MOTORCMD;
std::mutex MTX_MOTORSTATES;
std::mutex MTX_IMU;

std::mutex MTX_SPICMD;
std::mutex MTX_SPIREC;

Vec3 RPY_DES;
Vec3 POS_WORLD_DES;
Vec3 VEL_WORLD_DES;

CPG _CPG;

int GAIT_INDEX = 0;

int GAIT_SCHEDULER_SWITCH_INDEX = 0;

KeyBoard::KeyBoard(){
    userCmd = UserCommand::NONE;
    userValue.setZero();

    tcgetattr( fileno( stdin ), &_oldSettings );
    _newSettings = _oldSettings;
    _newSettings.c_lflag &= (~ICANON & ~ECHO);
    tcsetattr( fileno( stdin ), TCSANOW, &_newSettings );

    pthread_create(&_tid, NULL, runKeyBoard, (void*)this);
    printf(" KeyBoard checkCmd:\n 1->PASSIVE_1 ( ----******---- );\n 2->FIXEDSTAND_2 ( ----******---- );\n c->FIXEDSQUAT_c ( ----******---- );\n 3->FREESTAND_3;\n 4->QP_4 ( ----******---- );\n 5->POSITION_5 ( ----******---- );\n 6->A1MPC_6(Ban);\n 7->POSREFLEX_7(Ban);\n 8->FORCE_POS ( ----******---- );\n 9->SWING_TEST9\n; 0->MPC_FOREC_POS0 ( ----******---- )\n");
    printf(" TERRIANESTI_FOURLEG: %d \n",TERRIANESTI_FOURLEG);
}

KeyBoard::~KeyBoard(){
    pthread_cancel(_tid);
    pthread_join(_tid, NULL);
    tcsetattr( fileno( stdin ), TCSANOW, &_oldSettings );
}

UserCommand KeyBoard::checkCmd(){
    // printf("\n-a-gf-asg-\n");
    switch (_c){
    case '1':
        return UserCommand::PASSIVE_1;
    case 'c':
        FORCE_PROTECT_CHANGE = false;
        return UserCommand::SQUAT_C;
    case '2':
        FORCE_PROTECT_CHANGE = false;
        return UserCommand::FIXEDSTAND_2;
    case '3':
        FORCE_PROTECT_CHANGE = false;
        return UserCommand::FREESTAND_3;
    case '5':
        FORCE_PROTECT_CHANGE = false;
        return UserCommand::POSITION_5;
    case '9':
        return UserCommand::SWING_TEST9;
    case '8':
        FORCE_PROTECT_CHANGE = true;
        return UserCommand::FORCE_POS_8;
    case '7':
        FORCE_PROTECT_CHANGE = true;
        return UserCommand::FORCE_POS_WBC_7;
    case ' ':
        {   
            USVLCC_SETZERO = true;
            // userValue.LTsetZero = true;
            // userValue.setZero();
            // printf(" space\n");
        }
        return UserCommand::NONE;
    default:
        return UserCommand::NONE;
    }
}

void KeyBoard::changeValue(){
    switch (_c){
    // case 'w':case 'W':
    case 'w':
        userValue.ly = min<float>(userValue.ly+sensitivityLeft, 1.0);
        userValue_lcc.ly = min<float>(userValue_lcc.ly+sensitivityLeft, 1.0);
    break;
    // case 's':case 'S':
    case 's':
        userValue.ly = max<float>(userValue.ly-sensitivityLeft, -1.0);
        userValue_lcc.ly = max<float>(userValue_lcc.ly-sensitivityLeft, -1.0);
        break;
    // case 'd':case 'D':
    case 'd':
        userValue.lx = min<float>(userValue.lx+sensitivityLeft, 1.0);
        userValue_lcc.lx = min<float>(userValue_lcc.lx+sensitivityLeft, 1.0);
        break;
    // case 'a':case 'A':
    case 'a':
        userValue.lx = max<float>(userValue.lx-sensitivityLeft, -1.0);
        userValue_lcc.lx = max<float>(userValue_lcc.lx-sensitivityLeft, -1.0);
        break;

    // case 'i':case 'I':
    case 'i':
        userValue.ry = min<float>(userValue.ry+sensitivityRight, 1.0);
        userValue_lcc.ry = min<float>(userValue_lcc.ry+sensitivityRight, 1.0);
        break;
    // case 'k':case 'K':
    case 'k':
        userValue.ry = max<float>(userValue.ry-sensitivityRight, -1.0);
        userValue_lcc.ry = max<float>(userValue_lcc.ry-sensitivityRight, -1.0);
        break;
    // case 'l':case 'L':
    case 'l':
        userValue.rx = min<float>(userValue.rx+sensitivityRight, 1.0);
        userValue_lcc.rx = min<float>(userValue_lcc.rx+sensitivityRight, 1.0);
        break;
    // case 'j':case 'J':
    case 'j':
        userValue.rx = max<float>(userValue.rx-sensitivityRight, -1.0);
        userValue_lcc.rx = max<float>(userValue_lcc.rx-sensitivityRight, -1.0);
        break;
    default:
        break;
    }
}

float raddd = 3.14159/180;
void KeyBoard::changeFunctionModeValue(){
    switch (_c){
        case 'M':case 'm':{ //进入闭环
        if( KEY_M == false )
            KEY_M = true;
        else if( KEY_M == true )
            KEY_M = false;
        std::cout<<"KEY_M:  "<< KEY_M <<std::endl;
        }
        break; 
        #if USE_A_REAL_HEXAPOD == true
        case 'p':case 'P':{ //进入闭环
                // if( userFunctionMode.motor_enable_flag == false )
                //     userFunctionMode.motor_enable_flag = true;
                // else if( userFunctionMode.motor_enable_flag == true )
                //     userFunctionMode.motor_enable_flag = false;
                // std::cout<<"motor_enable_flag:  "<< userFunctionMode.motor_enable_flag <<std::endl;

                MOTOR_ENABLE_FLAG = true;
                MOTOR_DISABEL_FLAG = false;
            }
            break;
        case 'o':case 'O':{ //退出闭环
                // std::cout<<"getQ_Hex:  \n"<< _ctrlComp->lowState->getQ_Hex() * 180/3.1415926 <<std::endl;
                // spi_2.exit_close_loop();

                MOTOR_DISABEL_FLAG = true;
                MOTOR_DATA_LOAD = false;

                // if( userFunctionMode.motor_disenable_flag == false )
                //     userFunctionMode.motor_disenable_flag = true;
                // else if( userFunctionMode.motor_disenable_flag == true )
                //     userFunctionMode.motor_disenable_flag = false;
                // std::cout<<"motor_disenable_flag:  "<< userFunctionMode.motor_disenable_flag <<std::endl;
            }
            break;
        case '[':case '{':{ //电机连续加载数据
                MOTOR_READY_FLAG = true;
                MOTOR_DATA_LOAD = false;
                MOTOR_DISABEL_FLAG = false;
            }
            break;
        case ']':case '}':{ //进入程序算法，点击获得程序的控制数据
                MOTOR_DATA_LOAD = true;
                MOTOR_READY_FLAG = false;
                printf(" \n  ----------------- fsm_run -------------------- \n ");
            }
            break;
        case '-':case '_':{ //退出闭环
                DOU_DONG_ANGEL = DOU_DONG_ANGEL -1 * raddd;
                if( DOU_DONG_ANGEL >= 30 * raddd)
                    DOU_DONG_ANGEL = 30 * raddd;
                else if( DOU_DONG_ANGEL <= -30 * raddd )
                    DOU_DONG_ANGEL = -30 * raddd;
            }
            break;
        case '=':case '+':{ //退出闭环
                DOU_DONG_ANGEL = DOU_DONG_ANGEL +1 * raddd;
                if( DOU_DONG_ANGEL >= 30 * raddd)
                    DOU_DONG_ANGEL = 30 * raddd;
                else if( DOU_DONG_ANGEL <= -30 * raddd )
                    DOU_DONG_ANGEL = -30 * raddd;
            }
            break;
        #endif
        case 't':case 'T':{

            if( DTAT_SAVE2TXT == false )
                DTAT_SAVE2TXT = true;
            else if( DTAT_SAVE2TXT == true )
                DTAT_SAVE2TXT = false;
            std::cout<<"DTAT_SAVE2TXT:  "<< DTAT_SAVE2TXT <<std::endl;
        }
        break;
        case 'G':case 'g':{
            USVLCC_SETZERO = true;
            if( GAIT_SCHEDULER_SWITCH_INDEX == 0 )
            GAIT_SCHEDULER_SWITCH_INDEX = 1; //1 -> CPG
            else if( GAIT_SCHEDULER_SWITCH_INDEX == 1 )
            GAIT_SCHEDULER_SWITCH_INDEX = 0; // 0 -> MIT
            std::cout<<"GAIT_SCHEDULER_SWITCH_INDEX:  "<< GAIT_SCHEDULER_SWITCH_INDEX <<std::endl;
        }
        break;

        case ',':case '<':{
            GAIT_INDEX = 0;
            std::cout<<"GAIT_INDEX:  "<< GAIT_INDEX <<std::endl;
        }
        break;
        case '.':case '>':{
            GAIT_INDEX = 1;
            std::cout<<"GAIT_INDEX:  "<< GAIT_INDEX <<std::endl;
        }
        break;
        case '/':case '?':{
            GAIT_INDEX = 2;
            std::cout<<"GAIT_INDEX:  "<< GAIT_INDEX <<std::endl;
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