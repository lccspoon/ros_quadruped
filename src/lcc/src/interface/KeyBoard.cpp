 
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
        return UserCommand::POSITION_5;

#ifdef COMPILE_WITH_MOVE_BASE
    case '5':
        return UserCommand::L2_Y;
#endif  // COMPILE_WITH_MOVE_BASE
    case '6':
        return UserCommand::A1MPC_6;
    case '7':
        return UserCommand::POSREFLEX_7;
    case '0':
        return UserCommand::BALANCE_TEST0;
    case '9':
        return UserCommand::SWING_TEST9;
    case '8':
        return UserCommand::SETP_TEST8;
    case ' ':
        {
            userValue.setZero();
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
        break;
    // case 's':case 'S':
    case 's':
        userValue.ly = max<float>(userValue.ly-sensitivityLeft, -1.0);
        break;
    // case 'd':case 'D':
    case 'd':
        userValue.lx = min<float>(userValue.lx+sensitivityLeft, 1.0);
        break;
    // case 'a':case 'A':
    case 'a':
        userValue.lx = max<float>(userValue.lx-sensitivityLeft, -1.0);
        break;

    // case 'i':case 'I':
    case 'i':
        userValue.ry = min<float>(userValue.ry+sensitivityRight, 1.0);
        break;
    // case 'k':case 'K':
    case 'k':
        userValue.ry = max<float>(userValue.ry-sensitivityRight, -1.0);
        break;
    // case 'l':case 'L':
    case 'l':
        userValue.rx = min<float>(userValue.rx+sensitivityRight, 1.0);
        break;
    // case 'j':case 'J':
    case 'j':
        userValue.rx = max<float>(userValue.rx-sensitivityRight, -1.0);
        break;
    default:
        break;
    }
}

// lcc 20250601
int life_reaction_off_on_flag=0,dowm_reaction_off_on_flag=0,mkan_reaction_off_on_flag=0, berzier_shape_off_on_flag = 0;
void KeyBoard::changeFunctionModeValue(){
    switch (_c){
    case 't':case 'T':{
            if( userFunctionMode.function_test == false )
                userFunctionMode.function_test = true;
            else if( userFunctionMode.function_test == true )
                userFunctionMode.function_test = false;
            std::cout<<"function_test:  "<< userFunctionMode.function_test <<std::endl;
            if( userFunctionMode.state_reset == false )
                userFunctionMode.state_reset = true;
            else if( userFunctionMode.state_reset == true )
                userFunctionMode.state_reset = false;
            std::cout<<"state_reset:  "<< userFunctionMode.state_reset <<std::endl;
        }
        break;
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