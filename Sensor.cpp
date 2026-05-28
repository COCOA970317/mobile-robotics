#include "All.h"

frc::AnalogInput *Sensor::psd[Constants::PSD_CNT];
frc::Ultrasonic *Sensor::ping[Constants::PING_CNT];
AHRS Sensor::gyro{frc::SPI::Port::kMXP, 200};
studica::Cobra Sensor::cobra{};
frc::DigitalOutput Sensor::led_g{Constants::LED_GREEN}, Sensor::led_r{Constants::LED_RED};

void Sensor::PSD_Loop(){
    clock_t past;
    const int ave_cnt=8;
    double psd_raw[Constants::PSD_CNT][ave_cnt]={0,}, sum;
    int cnt=0;

    for(int i=0;i<Constants::PSD_CNT;i++) Sensor::psd[i] = new frc::AnalogInput(Constants::PSD[i]);

    while(true){
        past = clock();

        for(int i=0;i<Constants::PSD_CNT;i++){
            psd_raw[i][cnt] = Sensor::psd[i]->GetVoltage();
            if(cnt>=ave_cnt-1){
                std::sort(&psd_raw[i][0],&psd_raw[i][ave_cnt]);

                sum=0;
                for(int j=0;j<cnt-1;j++){
                    psd_raw[i][j]=psd_raw[i][j+1];
                    sum+=psd_raw[i][j];
                }

                sum/=cnt-1;
                ::sensor[Constants::PSD_NUM[i]] = pow(sum, -1.2045) * 277.26 + Constants::PSD_ERR[i];
            }
        }
        if(cnt>=ave_cnt-1) cnt--;
        else cnt++;

        // if(clock()-past>=15000) cout << "PSD Timeout: " << clock()-past << endl;

        loop_delay(5, past);
    }
}
void Sensor::Ping_Loop(){
    clock_t past;
    // 1. 센서 객체 생성 및 초기화
    for(int i=0; i<Constants::PING_CNT; i++) {
        Sensor::ping[i] = new frc::Ultrasonic(Constants::PING[i][0], Constants::PING[i][1]);
        Sensor::ping[i]->SetEnabled(true);
    }

    // 2. [핵심] 자동 모드 활성화 
    // (모든 초음파 센서가 라운드 로빈 방식으로 자동 순환하며 가장 빠르게 측정합니다)
    frc::Ultrasonic::SetAutomaticMode(true);

    while(true){
        past = clock();
        
        for(int i=0; i<Constants::PING_CNT; i++){ // i<2 대신 PING_CNT 사용 권장
            
            // 3. 값 읽기 (Ping()이나 delay 필요 없음. 그냥 읽으면 최신 값임)
            if (Sensor::ping[i]->IsRangeValid()) {
                double current_range = Sensor::ping[i]->GetRangeMM();
                
                // 4. [보정] 노이즈 방지 (0이거나 터무니없는 값 무시)
                if (current_range > 0) {
                     ::sensor[Constants::PING_NUM[i]] = current_range + Constants::PING_ERR[i];
                }
            }
        }
        
        // 너무 자주 읽어서 CPU를 점유하지 않도록 최소한의 딜레이만 줌
        // (센서 측정 속도에는 영향을 주지 않음)
        loop_delay(5, past);
    }
}
void Sensor::Gyro_Loop(){
    clock_t past;
    double cycle=0;

    const double gain[2][4]={
        {0, 0.5, 3, 20},
        {0, 1, 5, 70}
    };

    while(true){
        past = clock();
        
        ::n_yaw = Sensor::gyro.GetYaw();

        for(int i=0;i<4;i++){
            if(i==0){
                cycle = ::n_yaw-::p_yaw[i];
                if(cycle<-180) cycle+=360;
                else if(cycle>=180) cycle-=360;
                ::yaw[i]+=cycle;
                ::p_yaw[i]=::n_yaw;

                cycle = -::yaw[i];
            }
            else{
                if(::n_yaw-::p_yaw[i]<0) ::yaw[i]=::n_yaw-::p_yaw[i] + 360;
                else ::yaw[i] = ::n_yaw-::p_yaw[i];

                if(::gyro_heading) cycle = ::gyro_heading - ::yaw[i];
                else cycle = ::input_w[i] - ::yaw[i];
                
                if(cycle>180) cycle-=360;
                else if(cycle<-180) cycle+=360;
            }
            ::gyro[i] = Trans(Arr(gain), fabs(cycle)) * sign(cycle);
        }
        Move::position(0);
        
        delay(30 - (clock()-past)/1000, false);
        loop_delay(10, past);
    }
}