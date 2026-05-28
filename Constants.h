#pragma once

enum{ // 탈출 조건 (num 인자값에 사용)
    LB=0, // Left Back (현재 사용 X)

    LF=1, // Left Front (현재 Ultra Left)
    FL=2, // Front Left (현재 PSD Left)
    FR=3, // Front Right (현재 PSD Right)
    RF=4, // Right Front (현재 Ultra Right)

    BS=5, // Back Sensor (현재 PSD Back)
    
    FLR=6, // Front Left, Right (센서 두 개 중 하나라도 가까워진다면)

    COB=8, // 라인 감지
    COB_LOST=9, // 라인 사라짐
    DIS=10, // 거리
    XLF=11,
    XFL=12,
    XFR=13,
    XRF=14,
    XBS=15,
};

enum{OMS_Z=1, WAIT, GRIPPER, GRIPPER_TURN, SERVO_TURN}; // OMS Task

namespace Constants
{
    static constexpr double MOTOR_RPM = 100.0;           // Maverick 61:1
    static constexpr double WHEEL_DIAMETER_MM = 100.0;   // 옴니휠
    static constexpr double WHEEL_CIRCUMFERENCE = M_PI * WHEEL_DIAMETER_MM; // 314.159mm
    
    // 로봇 최대 속도 계산
    // 100 RPM = 1.67 RPS → 1.67 * 314.159mm = 523.6 mm/s
    static constexpr double MAX_SPEED_MM_PER_SEC = (MOTOR_RPM / 60.0) * WHEEL_CIRCUMFERENCE; // ~524 mm/s
    
    // 코드에서 사용하는 속도 범위가 -534~534인 이유:
    // 534는 최대 물리 속도(524 mm/s)를 약간 초과하는 안전 여유값
    
    // 엔코더 해상도
    static constexpr int ENCODER_CPR = 1464; // counts per revolution
    static constexpr double MM_PER_ENCODER_COUNT = WHEEL_CIRCUMFERENCE / ENCODER_CPR; // 0.2146 mm
    
    // 3륜 홀로노믹 기하학 (로봇 중심에서 휠까지 거리)
    static constexpr double ROBOT_RADIUS_MM = 203.633; // 실측 필요! (중요!)
    
    // 회전 상수 계산 (현재 코드: w*3.4)
    // 이론값: ROBOT_RADIUS_MM / (WHEEL_DIAMETER_MM / 2)
    static constexpr double ROTATION_FACTOR = ROBOT_RADIUS_MM / (WHEEL_DIAMETER_MM / 2.0); // ~3.4
    
    enum{H, L}; // Titan 스위치 단자 High, Low

    static constexpr int TIMEOUT_SEC = 15; // 타임아웃시간

    static constexpr int LED_GREEN = 16; // 초록 LED 핀번호
    static constexpr int LED_REDs = 14; // 빨강 LED   핀번호

    static constexpr int EMS[2] = {2, L}; // 비상정지 핀번호
    static constexpr int SWITCH[2][2] = {{0, H}, {1, H}}; // 스위치 핀번호 (Titan에 연결)
    static constexpr int LIMIT_SW_CNT = 5; // 사용할 리밋 스위치 총 개수

    // {Titan 모터 번호(0~3), 단자(H 또는 L)}
    // 물리적인 핀 연결 상태에 맞춰 앞의 번호와 뒤의 단자를 수정해서 사용하세요.
    static constexpr int LIMIT_SW[LIMIT_SW_CNT][2] = {
        {1, L}, // 0번 스위치 - 기존 OMS_Z_LIMIT 위치 DC모터(3)
        {3, L}, // 1번 스위치 - 스크류 서보모터(3)
        {3, H}, // 2번 스위치 - 그리퍼 올리기(2)
        {0, L}, // 3번 스위치 - 그리퍼 내리기(2)
        {2, H}  // 4번 스위치 - 그리퍼 놓기(1)
    };

    static constexpr int PSD_CNT = 3; // PSD 센서 개수
    static constexpr int PSD[PSD_CNT] = {0, 2, 3}; // PSD 센서 핀번호
    static constexpr int PSD_NUM[PSD_CNT] = {FL, FR, BS}; // PSD 센서 위치 지정
    static constexpr double PSD_ERR[PSD_CNT] = {0.6, 0, 0}; // PSD 센서 오차 값

    static constexpr int PING_CNT = 2; // Ultra 센서 개수
    static constexpr int PING[PING_CNT][2] = {{13, 10}, {12, 9}}; // 초음파센서 핀번호 (Trigger, Echo) (출력, 입력)
    static constexpr int PING_NUM[PING_CNT] = {LF, RF}; // 초음파센서 위치 지정
    static constexpr double PING_ERR[PING_CNT] = {0, 0}; // 초음파센서 오차 값
    
    static constexpr int COBRA_SENSITIVITY = 2040; // 코브라센서 감지 값

    static constexpr int MOTOR[4] = {0, 1, 2, 3}; // 모터 번호 (Titan)
    static constexpr int ENCODER[4][2] = {{0, 1}, {2, 3}, {4, 5}, {6, 7}}; // 엔코더 핀번호
    static constexpr double PID_K[3] = {1.5, 0.25, 0}; // PID Gain 값
    static constexpr double OMS_PID_K[3] = {0, 0, 0}; // 상하축 PID Gain 값

    // static constexpr double OMS_Z_TRANS[2][2] = {{-1485, 233}, {28, 393}}; // 상하축 엔코더 값 -> mm로 변환
    static constexpr double OMS_Z_TRANS[2][2] = {{-1127, -10}, {-4, 108}}; // 상하축 엔코더 값 -> mm로 변환

    static constexpr int SERVO_CNT = 5; // 서보모터 개수
    static constexpr int SERVO[SERVO_CNT] = {17,18,19,20,21}; // 서보모터 핀번호 (위에 OMS_Task와 일치시켜야함)
 
    static constexpr double SERVO_OFFSET[SERVO_CNT] = {90.0, 150.0, 150.0, 150.0, 60.0};
    //17 = , 18 = , 19 = , 20 = 공 분배기, 21 = 공 분류기

    static constexpr double OMS_SET = 0;
    static constexpr double ROL_SET = 143;
    static constexpr double BBuZZiBBuZZiBBaBBa = 0;
    static constexpr double OMS_GRIPPER_CLOSE = 0; // 그리퍼 닫는 값
    static constexpr double OMS_GRIPPER_TURN_CENTER = 150+1; // 그리퍼 회전축 가운데 값
}