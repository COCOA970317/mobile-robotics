#include "All.h"
#include <frc/DriverStation.h> // [추가 1] 배터리 전압을 읽기 위해 헤더 추가

studica::TitanQuad Motor::motor[4]={
    {42, 20000, Constants::MOTOR[0]},
    {42, 20000, Constants::MOTOR[1]},
    {42, 20000, Constants::MOTOR[2]},
    {42, 20000, Constants::MOTOR[3]},
};

frc::Encoder Motor::encoder[4]={
    frc::Encoder(Constants::ENCODER[0][0], Constants::ENCODER[0][1]), 
    frc::Encoder(Constants::ENCODER[1][0], Constants::ENCODER[1][1]), 
    frc::Encoder(Constants::ENCODER[2][0], Constants::ENCODER[2][1]), 
    frc::Encoder(Constants::ENCODER[3][0], Constants::ENCODER[3][1]), 
};

frc2::PIDController Motor::pid[4]={
    frc2::PIDController(Constants::PID_K[0], Constants::PID_K[1], Constants::PID_K[2], units::second_t(10_ms)),
    frc2::PIDController(Constants::PID_K[0], Constants::PID_K[1], Constants::PID_K[2], units::second_t(10_ms)),
    frc2::PIDController(Constants::PID_K[0], Constants::PID_K[1], Constants::PID_K[2], units::second_t(10_ms)),
    frc2::PIDController(Constants::OMS_PID_K[0], Constants::OMS_PID_K[1], Constants::OMS_PID_K[2], units::second_t(10_ms)),    
};

studica::Servo *Motor::servo[Constants::SERVO_CNT];

void Motor::Reset(int &step){
    ::led_g = 0;
    ::led_r = 1;

    if(step==0){
        ::motor[3]=0.22;
        if(::oms_z_limit){
            ::motor[3]=-0.05;
            step=1;
        }
    }
    else if(step==1){
        ::motor[3]=-0.05;
        
        if(!::oms_z_limit){ 
            ::motor[3]=0;
            Sensor::gyro.Reset();
            ShuffleBoard::nt_mode.SetDouble(0);
            for(int i=0;i<4;i++){
                Motor::encoder[i].Reset();
                ::yaw[i] = 0;
                ::p_yaw[i] = ::n_yaw; // 현재 자이로 값을 기준점으로
                ::input_w[i] = 0;
            }
            step=2;
        }
    }
}
int Motor::Find_Titan_Number(int num){
    for(int i=0;i<4;i++) if(num==Constants::MOTOR[i]) return i;
    return 0;
}

void Motor::Init(){
    for(int i=0;i<Constants::SERVO_CNT;i++) servo[i] = new studica::Servo(Constants::SERVO[i] - 12);

    Motor::motor[0].SetInverted(false);
    Motor::motor[1].SetInverted(false);
    Motor::motor[2].SetInverted(false);
    Motor::motor[3].SetInverted(false);

    Motor::encoder[0].SetReverseDirection(false);
    Motor::encoder[1].SetReverseDirection(false);
    Motor::encoder[2].SetReverseDirection(false);
    Motor::encoder[3].SetReverseDirection(false);
}
void Motor::Loop(){
    int EMS_NUM = Find_Titan_Number(Constants::EMS[0]);
    int SW1_NUM = Find_Titan_Number(Constants::SWITCH[0][0]);
    int SW2_NUM = Find_Titan_Number(Constants::SWITCH[1][0]);
    //int OMS_Z_LIMIT_NUM = Find_Titan_Number(Constants::OMS_Z_LIMIT[0]);
    
    int ems_flg=1, step=0;
    clock_t past;

    double prev_en[4]={0,}, difference[4]={0,};
    double err[4]={0,};

    int motor_reset_check = 0;
    
    // [추가 2] 기준 전압 설정 (11.0V를 추천합니다. 너무 낮으면 로봇이 느려집니다)
    const double TARGET_VOLTAGE = 11.4; 

    Init();

    while(true){
        past = clock();

        // [추가 3] 현재 배터리 전압을 읽고 보정 비율(comp_ratio) 계산
        double battery_voltage = frc::DriverStation::GetInstance().GetBatteryVoltage();
        double comp_ratio = 1.0; 

        // 전압이 8V 이상일 때만 계산 (센서 오류 방지)
        if (battery_voltage > 8.0) {
            comp_ratio = TARGET_VOLTAGE / battery_voltage;
            // 배터리가 11V보다 낮다고 힘을 키워줄 순 없으므로, 최대 1.0으로 제한
            if(comp_ratio > 1.0) comp_ratio = 1.0;
        }

        if(!::Disable && ems_flg){
            Reset(step);

            if(step == 2) { 
               for(int i=0; i<4; i++) prev_en[i] = 0;
            }
        }

        for(int i=0;i<4;i++){
            ::encoder[i] = Motor::encoder[i].GetDistance();

            difference[i] = ::encoder[i] - prev_en[i];
            err[i] = Trans(-24,24,-1,1,difference[i]);
            err[i] = pid[i].Calculate(err[i], ::motor[i]);

            prev_en[i]=::encoder[i];
        }
        ::oms_z_now = Trans2(Constants::OMS_Z_TRANS[0][0],Constants::OMS_Z_TRANS[0][1],Constants::OMS_Z_TRANS[1][0],Constants::OMS_Z_TRANS[1][1],::encoder[3]);

        if(ems_flg && step==2){
            ::Motor_Reset = true;
            ems_flg=0, step=0;
            for(int i=0;i<4;i++){
                pid[i].Reset();
                prev_en[i]=0;
            }
            // // ▼▼ [핵심 추가] EMS가 풀리고 로봇이 켜질 때, 모든 서보모터를 기본 안전 각도로 이동 ▼▼
            // for(int i=0; i<Constants::SERVO_CNT; i++) {
            //     OMS::set_servo(i, Constants::SERVO_OFFSET[i], 0);
            // }
            ::led_r = 1;
        }

        ::ems = motor[EMS_NUM].GetLimitSwitch(Constants::EMS[1]);
        
        // 새로 추가한 5개 리밋 스위치 전체 업데이트 함수 실행
        LimitSW::Update();
        
        // 기존 상하축(Z축) 리밋 변수에는 0번 스위치의 누름 상태를 전달
        ::oms_z_limit = LimitSW::is_holding(0); 
        
        ::sw1 = !motor[SW1_NUM].GetLimitSwitch(Constants::SWITCH[0][1]);
        ::sw2 = !motor[SW2_NUM].GetLimitSwitch(Constants::SWITCH[1][1]);

        if(::ems){
            for(int i=0;i<4;i++){
                Motor::motor[i].Set(0);
                ::motor[i]=0;
            }
            for(int i=0;i<Constants::SERVO_CNT;i++) Motor::servo[i]->SetOffline();

            if(!ems_flg){
                ems_flg=1;
            }
            pid[3].Reset();
            M_cnt = 0;
        }
        else{
            if(::Motor_Reset){
                for(int i=0;i<3;i++){
                    Motor::motor[i].Set(0);
                    err[i]=0;
                    ::motor[i]=0;
                }
                for(int i=0;i<3;i++){
                    pid[i].Reset();
                }

                motor_reset_check++;
                if(motor_reset_check>=2){
                    motor_reset_check=0;
                    ::Motor_Reset = false;
                }
            }
            else{
                for(int i=0;i<3;i++){
                    if(!::pid_flg || ::Disable) err[i]=0;
                    ::pid_speed[i]=::motor[i]+err[i];

                    // [수정 1] 구동 모터(0,1,2)에 전압 보정 적용
                    // 계산된 속도에 comp_ratio를 곱해서, 배터리가 쎌 때는 힘을 줄임
                    double final_speed = ::pid_speed[i] * comp_ratio;
                    Motor::motor[i].Set(std::clamp(final_speed, -1.0, 1.0));
                }
            }
            if(pid_flg==0) err[3]=0;
            ::pid_speed[3]=::motor[3]+err[3];
            
            // [수정 2] 리프트 모터(3)에도 전압 보정 적용 (원치 않으면 comp_ratio 제거)
            double final_lift_speed = ::pid_speed[3] * comp_ratio;
            Motor::motor[3].Set(std::clamp(final_lift_speed, -1.0, 1.0));

            // ▼▼▼ [오류 수정 부분] 중첩된 for문을 제거하고 하나로 합칩니다 ▼▼▼
            for(int i=0; i<Constants::SERVO_CNT; i++){
                if(::servo[i] < 0) {
                    Motor::servo[i]->SetOffline(); // -1 등 음수값이면 서보 끄기
                } 
                else {
                    Motor::servo[i]->SetAngle(::servo[i]); 
                }
            }
        }
        
        if(clock()-past>=15000) cout << "Motor Timeout: " << clock()-past << endl;

        loop_delay(10, past);
    }
    
}

namespace LimitSW {
    bool current[Constants::LIMIT_SW_CNT] = {false, };
    bool prev[Constants::LIMIT_SW_CNT] = {false, };

    void Update() {
        for(int i = 0; i < Constants::LIMIT_SW_CNT; i++) {
            // 1. 현재 값을 과거 값으로 밀어내기 저장
            prev[i] = current[i];
            
            // 2. Constants에 등록된 Titan 번호(CAN ID)의 실제 인덱스 찾기
            int motor_idx = Motor::Find_Titan_Number(Constants::LIMIT_SW[i][0]);
            
            // 3. Titan 보드에서 해당 단자(H/L)의 리밋 스위치 상태 읽어오기
            // (! 기호는 스위치가 눌렸을 때 전기적으로 0(Low)이 들어오는 것을 true로 뒤집기 위함)
            current[i] = !Motor::motor[motor_idx].GetLimitSwitch(Constants::LIMIT_SW[i][1]);
        }
    }

    // 상태 판별 함수들
    bool is_pressed(int idx) { return current[idx] == true && prev[idx] == false; }
    bool is_released(int idx) { return current[idx] == false && prev[idx] == true; }
    bool is_holding(int idx) { return current[idx] == true; }
}