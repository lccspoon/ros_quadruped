#include "interface/KeyBoard.h"
// #include "interface/ "
#include <iostream>
#include "interface/IOSDK.h"

bool KEY_M = false;
bool USVLCC_SETZERO = false;
bool FORCE_PROTECT_CHANGE = false;

std::mutex MTX_MOTORCMD;
std::mutex MTX_MOTORSTATES;
std::mutex MTX_IMU;

std::mutex MTX_SPICMD;
std::mutex MTX_SPIREC;


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
    case '4':
        // printf(" \n keyboard->QP_4  \n ");
        return UserCommand::QP_4;
    case '5':
        FORCE_PROTECT_CHANGE = false;
        return UserCommand::POSITION_5;

    #ifdef COMPILE_WITH_MOVE_BASE
        case '5':
            return UserCommand::L2_Y;
    #endif  // COMPILE_WITH_MOVE_BASE

    case '6':
        return UserCommand::A1MPC_6;
    // case '7':
    //     return UserCommand::POSREFLEX_7;
    // case '0':
    //     return UserCommand::BALANCE_TEST0;
    case '0':
        return UserCommand::MPC_FORCE_POS_0;
    case '9':
        return UserCommand::SWING_TEST9;
    // case '8':
    //     return UserCommand::SETP_TEST8;
    case '8':
        FORCE_PROTECT_CHANGE = true;
        // printf(" FORCE_PROTECT_CHANGE:%d\n",FORCE_PROTECT_CHANGE);
        return UserCommand::FORCE_POS_8;
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

// lcc 20250601
int life_reaction_off_on_flag=0,dowm_reaction_off_on_flag=0,mkan_reaction_off_on_flag=0, berzier_shape_off_on_flag = 0;
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
                if( DOU_DONG_ANGEL >= 2.5 * raddd)
                    DOU_DONG_ANGEL = 2.5 * raddd;
                else if( DOU_DONG_ANGEL <= -2.5 * raddd )
                    DOU_DONG_ANGEL = -2.5 * raddd;
            }
            break;
        case '=':case '+':{ //退出闭环
                DOU_DONG_ANGEL = DOU_DONG_ANGEL +1 * raddd;
                if( DOU_DONG_ANGEL >= 2.5 * raddd)
                    DOU_DONG_ANGEL = 2.5 * raddd;
                else if( DOU_DONG_ANGEL <= -2.5 * raddd )
                    DOU_DONG_ANGEL = -2.5 * raddd;
            }
            break;
        case 't':case 'T':{

                if( TEST_FLAG == false )
                    TEST_FLAG = true;
                else if( TEST_FLAG == true )
                    TEST_FLAG = false;

                // if( userFunctionMode.function_test == false )
                //     userFunctionMode.function_test = true;
                // else if( userFunctionMode.function_test == true )
                //     userFunctionMode.function_test = false;
                // std::cout<<"function_test:  "<< userFunctionMode.function_test <<std::endl;
                // if( userFunctionMode.state_reset == false )
                //     userFunctionMode.state_reset = true;
                // else if( userFunctionMode.state_reset == true )
                //     userFunctionMode.state_reset = false;
                // std::cout<<"state_reset:  "<< userFunctionMode.state_reset <<std::endl;
        }
        break;
        #endif
        /******************20230906自适应cheet按键******************/
        #if PCONTROL_REFLEX_LIFE_DOWM == true
        case 'Q':   {   // lf
                    if (userFunctionMode.life_reaction_off_on==1){
                        if ( userFunctionMode.LEG_LIFT_TRIGGER(1) == 0 )
                            userFunctionMode.LEG_LIFT_TRIGGER(1)=1;
                        else if ( userFunctionMode.LEG_LIFT_TRIGGER(1) == 1 )
                            userFunctionMode.LEG_LIFT_TRIGGER(1)=0;
                        printf("\n LEG_LIFT_TRIGGER 1:%f \n",userFunctionMode.LEG_LIFT_TRIGGER(1));
                    }
                    if (userFunctionMode.dowm_reaction_off_on==1){
                        if ( userFunctionMode.LEG_DOWM_TRIGGER(1) == 0 )
                            userFunctionMode.LEG_DOWM_TRIGGER(1)=1;
                        else if ( userFunctionMode.LEG_DOWM_TRIGGER(1) == 1 )
                            userFunctionMode.LEG_DOWM_TRIGGER(1)=0;
                        printf("\n LEG_DOWM_TRIGGER 1:%f \n",userFunctionMode.LEG_DOWM_TRIGGER(1));
                    }
                }
                break;
        case 'W':   {  // rf
                    if (userFunctionMode.life_reaction_off_on==1){
                        if ( userFunctionMode.LEG_LIFT_TRIGGER(0) == 0 )
                            userFunctionMode.LEG_LIFT_TRIGGER(0)=1;
                        else if ( userFunctionMode.LEG_LIFT_TRIGGER(0) == 1 )
                            userFunctionMode.LEG_LIFT_TRIGGER(0)=0;
                        printf("\n LEG_LIFT_TRIGGER 0:%f \n",userFunctionMode.LEG_LIFT_TRIGGER(0));
                    }
                    if (userFunctionMode.dowm_reaction_off_on==1){
                        if ( userFunctionMode.LEG_DOWM_TRIGGER(0) == 0 )
                            userFunctionMode.LEG_DOWM_TRIGGER(0)=1;
                        else if ( userFunctionMode.LEG_DOWM_TRIGGER(0) == 1 )
                            userFunctionMode.LEG_DOWM_TRIGGER(0)=0;
                        printf("\n LEG_DOWM_TRIGGER 0:%f \n",userFunctionMode.LEG_DOWM_TRIGGER(0));
                    }
                }
                break;
        case 'A':  {   //lm
                    if (userFunctionMode.life_reaction_off_on==1){
                        if ( userFunctionMode.LEG_LIFT_TRIGGER(3) == 0 )
                            userFunctionMode.LEG_LIFT_TRIGGER(3)=1;
                        else if ( userFunctionMode.LEG_LIFT_TRIGGER(3) == 1 )
                            userFunctionMode.LEG_LIFT_TRIGGER(3)=0;
                        printf("\n LEG_LIFT_TRIGGER 3:%f \n",userFunctionMode.LEG_LIFT_TRIGGER(3));
                    }
                    if (userFunctionMode.dowm_reaction_off_on==1){
                        if ( userFunctionMode.LEG_DOWM_TRIGGER(3) == 0 )
                            userFunctionMode.LEG_DOWM_TRIGGER(3)=1;
                        else if ( userFunctionMode.LEG_DOWM_TRIGGER(3) == 1 )
                            userFunctionMode.LEG_DOWM_TRIGGER(3)=0;
                        printf("\n LEG_DOWM_TRIGGER 3:%f \n",userFunctionMode.LEG_DOWM_TRIGGER(3));
                    }
                }
                break;
        case 'S': {   //rm
                    if (userFunctionMode.life_reaction_off_on==1){
                        if ( userFunctionMode.LEG_LIFT_TRIGGER(2) == 0 )
                            userFunctionMode.LEG_LIFT_TRIGGER(2)=1;
                        else if ( userFunctionMode.LEG_LIFT_TRIGGER(2) == 1 )
                            userFunctionMode.LEG_LIFT_TRIGGER(2)=0;
                        printf("\n LEG_LIFT_TRIGGER 2:%f \n",userFunctionMode.LEG_LIFT_TRIGGER(2));
                    }
                    if (userFunctionMode.dowm_reaction_off_on==1){
                        if ( userFunctionMode.LEG_DOWM_TRIGGER(2) == 0 )
                            userFunctionMode.LEG_DOWM_TRIGGER(2)=1;
                        else if ( userFunctionMode.LEG_DOWM_TRIGGER(2) == 1 )
                            userFunctionMode.LEG_DOWM_TRIGGER(2)=0;
                        printf("\n LEG_DOWM_TRIGGER 2:%f \n",userFunctionMode.LEG_DOWM_TRIGGER(2));
                    }
                }
                break;
        case 'Z':   {   //lb
                    if (userFunctionMode.life_reaction_off_on==1){
                        if ( userFunctionMode.LEG_LIFT_TRIGGER(5) == 0 )
                            userFunctionMode.LEG_LIFT_TRIGGER(5)=1;
                        else if ( userFunctionMode.LEG_LIFT_TRIGGER(5) == 1 )
                            userFunctionMode.LEG_LIFT_TRIGGER(5)=0;
                        printf("\n LEG_LIFT_TRIGGER 5:%f \n",userFunctionMode.LEG_LIFT_TRIGGER(5));
                    }
                    if (userFunctionMode.dowm_reaction_off_on==1){
                        if ( userFunctionMode.LEG_DOWM_TRIGGER(5) == 0 )
                            userFunctionMode.LEG_DOWM_TRIGGER(5)=1;
                        else if ( userFunctionMode.LEG_DOWM_TRIGGER(5) == 1 )
                            userFunctionMode.LEG_DOWM_TRIGGER(5)=0;
                        printf("\n LEG_DOWM_TRIGGER 5:%f \n",userFunctionMode.LEG_DOWM_TRIGGER(5));
                    }
                }
                break;
        case 'X': {   //rb
                    if (userFunctionMode.life_reaction_off_on==1){
                        if ( userFunctionMode.LEG_LIFT_TRIGGER(4) == 0 )
                            userFunctionMode.LEG_LIFT_TRIGGER(4)=1;
                        else if ( userFunctionMode.LEG_LIFT_TRIGGER(4) == 1 )
                            userFunctionMode.LEG_LIFT_TRIGGER(4)=0;
                        printf("\n LEG_LIFT_TRIGGER 4:%f \n",userFunctionMode.LEG_LIFT_TRIGGER(4));
                    }
                    if (userFunctionMode.dowm_reaction_off_on==1){
                        if ( userFunctionMode.LEG_DOWM_TRIGGER(4) == 0 )
                            userFunctionMode.LEG_DOWM_TRIGGER(4)=1;
                        else if ( userFunctionMode.LEG_DOWM_TRIGGER(4) == 1 )
                            userFunctionMode.LEG_DOWM_TRIGGER(4)=0;
                        printf("\n LEG_DOWM_TRIGGER 4:%f \n",userFunctionMode.LEG_DOWM_TRIGGER(4));
                    }
                }
                break;
        case '!':  { // lift_reaction开关  通过按键控制决定是否启用这个反应行为
                    if(life_reaction_off_on_flag==0)
                    {   
                        life_reaction_off_on_flag=1;
                        userFunctionMode.life_reaction_off_on=1;
                    }   
                    else if(life_reaction_off_on_flag==1)
                    {
                        life_reaction_off_on_flag=0;
                        userFunctionMode.life_reaction_off_on=0;
                    }
                    printf("life_reaction_off_on: %d\n ",userFunctionMode.life_reaction_off_on);
                    printf("dowm_reaction_off_on: %d\n ",userFunctionMode.dowm_reaction_off_on);
                    printf("mkan_reaction_off_on: %d\n ",userFunctionMode.mkan_reaction_off_on);
                    std::cout<<" LEG_DOWM_TRIGGER "<<std::endl;
                    std::cout<< userFunctionMode.LEG_DOWM_TRIGGER <<std::endl;
                }
                break;
        case '@': {  //dowm_reaction开关
                    if(dowm_reaction_off_on_flag==0)
                    {   
                        dowm_reaction_off_on_flag=1;
                        userFunctionMode.dowm_reaction_off_on=1;
                    }   
                    else if(dowm_reaction_off_on_flag==1)
                    {
                        dowm_reaction_off_on_flag=0;
                        userFunctionMode.dowm_reaction_off_on=0;
                    }
                    printf("life_reaction_off_on: %d\n ",userFunctionMode.life_reaction_off_on);
                    printf("dowm_reaction_off_on: %d\n ",userFunctionMode.dowm_reaction_off_on);
                    printf("mkan_reaction_off_on: %d\n ",userFunctionMode.mkan_reaction_off_on);
                    std::cout<<" LEG_DOWM_TRIGGER "<<std::endl;
                    std::cout<< userFunctionMode.LEG_DOWM_TRIGGER <<std::endl;
                }
                break;
        case '#': { //dowm_reaction开关
                    if(mkan_reaction_off_on_flag==0)
                    {   
                        mkan_reaction_off_on_flag=1;
                        userFunctionMode.mkan_reaction_off_on=1;
                    }   
                    else if(mkan_reaction_off_on_flag==1)
                    {
                        mkan_reaction_off_on_flag=0;
                        userFunctionMode.mkan_reaction_off_on=0;
                    }

                    printf("life_reaction_off_on: %d\n ",userFunctionMode.life_reaction_off_on);
                    printf("dowm_reaction_off_on: %d\n ",userFunctionMode.dowm_reaction_off_on);
                    printf("mkan_reaction_off_on: %d\n ",userFunctionMode.mkan_reaction_off_on);
                    std::cout<<" LEG_DOWM_TRIGGER "<<std::endl;
                    std::cout<< userFunctionMode.LEG_DOWM_TRIGGER <<std::endl;
                    userFunctionMode.LEG_DOWM_TRIGGER.setZero();
                }
                case '~': {   //全部复原
                    userFunctionMode.LEG_LIFT_TRIGGER.setZero();
                    userFunctionMode.LEG_DOWM_TRIGGER.setZero();
                    userFunctionMode.berzier_shape_off_on=0;
                    userFunctionMode.mkan_reaction_off_on=0;
                    userFunctionMode.dowm_reaction_off_on=0;
                    userFunctionMode.life_reaction_off_on=0;
                    userFunctionMode.set_pitch = 0;
                    printf("life_reaction_off_on: %d\n ",userFunctionMode.life_reaction_off_on);
                    printf("dowm_reaction_off_on: %d\n ",userFunctionMode.dowm_reaction_off_on);
                    printf("mkan_reaction_off_on: %d\n ",userFunctionMode.mkan_reaction_off_on);
                    printf("set_pitch:%f\n",userFunctionMode.set_pitch*3.1415/180);
                }
                case 'v':   {  
                    if(dowm_reaction_off_on_flag==0 || dowm_reaction_off_on_flag==1){   
                        userFunctionMode.set_pitch=userFunctionMode.set_pitch+1*3.1415/180;
                        printf("set_pitch:%f\n",userFunctionMode.set_pitch*3.1415/180);
                    }   
                }
                break;
                case 'b':   {  
                    if(dowm_reaction_off_on_flag==0 || dowm_reaction_off_on_flag==1){
                        userFunctionMode.set_pitch=userFunctionMode.set_pitch-1*3.1415/180;
                        printf("set_pitch:%f\n",userFunctionMode.set_pitch*3.1415/180);  
                    }   
                }
                break;
            #endif
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