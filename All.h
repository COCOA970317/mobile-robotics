#pragma once

#include <iostream>
#include <math.h>
#include <time.h>
#include <thread>
#include <chrono>

#include "Constants.h"
#include "Sensor.h"
#include "Motor.h"
#include "OMS.h"
#include "Move.h"
#include "ShuffleBoard.h"

using std::cout;
using std::endl;

#define Arr(x) &x[0][0], sizeof(x[0])/sizeof(double) // 배열 인자로 넘기기 편하게 매크로 지정

extern int led_g, led_r, ems, sw1, sw2, M_cnt, Disable, pid_flg, pid_flg2; // led_g, led_r -> led on,off / ems: 비상 정지 / sw1, sw2: 스위치 1,2 / M_cnt: 스위치 카운트 / Disable: 비활성화 / pid_flg, pid_flg2: pid 활성화 여부
extern double sensor[6]; // PSD & 초음파센서 값 저장
extern int cobra, cobra_raw[4]; // 코브라센서 값 저장

extern double motor[4], encoder[4], pid_speed[4]; // 모터, 엔코더, pid속도 값 저장
extern double px[4], py[4], pw[4], gyro[4], n_yaw, yaw[4], p_yaw[4]; // px, py, pw -> 좌표 데이터 / gyro: 자이로센서 보정 값 / n_yaw, yaw[4], p_yaw[4]: 자이로 센서 각도 값
extern int gyro_heading, input_w[4]; // gyro_heading: 자이로센서 기준 방향 / input_w[4]: 자동 계산 heading

extern int oms_z_limit; // 상하축 리밋스위치
extern double servo[Constants::SERVO_CNT]; // 서보모터

extern bool Motor_Reset; // 모터 리셋 여부
extern double oms_z_now; // 상하축 현재 위치(mm)

void All_Init(); // 전체 쓰레드 실행
void loop_delay(clock_t ms, clock_t past); // 정밀한 delay 함수
void delay(clock_t ms, bool ems_flg=true); // delay 함수
double Trans(double Min, double Max, double min, double max, double value); // 범위 변환 함수
double Trans(const double arr[], int size, double value); // 범위 변환 함수(배열 인자 사용)
double Trans2(double Min, double Max, double min, double max, double value); // 범위 변환 함수 (최소, 최대 제한 없음)

int GetPreciseTime();

template <typename T1>
inline int sign(const T1 n){ // 부호 반환 함수
    return n<0 ? -1 : 1;
}

namespace LimitSW {
    extern bool current[Constants::LIMIT_SW_CNT];
    extern bool prev[Constants::LIMIT_SW_CNT];

    void Update();
    bool is_pressed(int idx);  // 방금 막 눌리는 순간 (1회 인식)
    bool is_released(int idx); // 방금 막 떨어지는 순간 (1회 인식)
    bool is_holding(int idx);  // 현재 계속 누르고 있는 상태
}