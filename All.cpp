#include "All.h"
#include "MockDS.h"
#include <cameraserver/CameraServer.h>
int led_g, led_r, ems, sw1, sw2, sw3, M_cnt, Disable, pid_flg=1, pid_flg2=1;
double sensor[6];
int cobra, cobra_raw[4];

double motor[4], encoder[4], pid_speed[4];
double px[4], py[4], pw[4], gyro[4], n_yaw, yaw[4], p_yaw[4];
int gyro_heading, input_w[4];

int oms_z_limit;
double servo[Constants::SERVO_CNT];

bool Motor_Reset;
double oms_z_now;

MockDS ds{};

int GetPreciseTime()
{
    static std::chrono::high_resolution_clock::time_point start_time = 
        std::chrono::high_resolution_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::high_resolution_clock::now() - start_time).count();
}

void All_Init(){
    ::led_r = 1;
    ::led_g = 0;
    for(int i=0;i<Constants::SERVO_CNT;i++) ::servo[i]=-1;

    std::thread(ShuffleBoard::Loop).detach();
    std::thread(Sensor::PSD_Loop).detach();
    
    std::thread(Sensor::Ping_Loop).detach();
    std::thread(Sensor::Gyro_Loop).detach();
    std::thread(Motor::Loop).detach();
    std::thread(OMS::Loop).detach();

    ds.Start(); // 자동 Enable 활성화
}

int GetTime()
{
    static std::chrono::high_resolution_clock::time_point start_time = 
        std::chrono::high_resolution_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::high_resolution_clock::now() - start_time).count();
}


void loop_delay(clock_t ms, clock_t past){
    int start = GetTime();
    int target_time = start + ms;
    
    while(GetTime() < target_time) {
        int remaining = target_time - GetTime();
        if(remaining > 1) {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        } else {
            break;
        }
    }
}

void delay(clock_t ms, bool ems_flg){
    if(ems_flg && ms>10){
        int end_time = GetTime() + ms;
        while(GetTime() < end_time) {
            if(::ems) return;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    else {
        std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    }
}

double Trans(double Min, double Max, double min, double max, double value){
    double rv;

    if(value<Min) value=Min;
    if(value>Max) value=Max;

    rv=(value-Min)/(Max-Min);
    rv*=max-min;

    return rv+min;
}
double Trans(const double arr[], int size, double value){
    int dir = value<0 ? -1:1;
    value = fabs(value);

	if(value<arr[0]) value=arr[0];
	else if(value>arr[size-1]) value=arr[size-1];
	
	for(int i=0;i<size;i++){
		if(value>=arr[i] && value<=arr[i+1]){
			return Trans(arr[i],arr[i+1],arr[i+size],arr[i+size+1],value) * dir;
		}
	}
	return 0;
}

double Trans2(double Min, double Max, double min, double max, double value){
    double rv;

    rv=(value-Min)/(Max-Min);
    rv*=max-min;

    return rv+min;
}