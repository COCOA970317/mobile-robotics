#include "Robot.h"
#include <cameraserver/CameraServer.h>
#include <iostream>
#include "ShuffleBoard.h"
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::StartCompetition() {
    std::cout << ">>> CameraServer Init Start <<<" << std::endl;

    static cs::HttpCamera piCam("PiCamera", "http://10.21.30.2:1181/cam.mjpg");
    piCam.SetPixelFormat(cs::VideoMode::PixelFormat::kMJPEG);
    frc::CameraServer::GetInstance()->StartAutomaticCapture(piCam);
    
    std::cout << ">>> CameraServer Init Done <<<" << std::endl;

    int sw1_flg=0, sw2_flg=0;
    void (*MISSION[21])()={0,M1,M2,M3,M4,M5,M6,M7,M8,M9,M10,M11,M12,M13,M14,M15,M16,M17,M18,M19,M20};

    All_Init();

    bool auto_run = false;

    while(!m_exit){
        frc::SmartDashboard::PutNumber("SW1_Status", sw1); 
        frc::SmartDashboard::PutNumber("SW2_Status", sw2);
        frc::SmartDashboard::PutNumber("Mission_Count", M_cnt);
        
        if(!ems){
            if(!auto_run){
                Mission_Init();
                 // ★★★ 여기서 테스트하고 싶은 미션 함수를 직접 적어줍니다 (M1(), M2() 등) ★★★
                Move::stop();
                auto_run = true; // 실행 완료 체크 (이후 무한 반복 방지)
                std::cout << "********** Auto Mission End **********" << std::endl;
            }

            if(!sw1_flg && sw1) sw1_flg=1;
            else if(sw1_flg && !sw1) sw1_flg=0, M_cnt++;

            if(!sw2_flg && sw2) sw2_flg=1;
            else if(sw2_flg && !sw2){
                sw2_flg=0;
                Move::position(0);
                Move::set_gyro(0);
                ::led_g=1;
                Mission_Init();
                
                if(M_cnt==0) All();
                else if(M_cnt>0 && M_cnt<21) (*(MISSION+M_cnt))();
                
                M_cnt=0;
                ::led_g=0;
                Move::stop();
                cout << "********** Mission End **********" << endl;
            }
        }
        delay(20, false);
    }
}
void Robot::EndCompetition(){ m_exit=true; }
#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif