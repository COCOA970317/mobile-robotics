#pragma once

namespace OMS{
    void Loop();

    // wait: 0 -> 안기다리고 바로 넘어감
    // wait: 1 -> OMS_Loop에서만 기다림
    // wait: 2 -> Main_Loop, OMS_Loop에서 작업이 끝날때까지 기다림

    void end(int wait=2); // OMS 전체 작업들 다 끝날때까지 기다림
    bool isFinished(int num=0); // OMS 작업 끝났는지 확인 (num=0이면 전체)
    void set_speed(double sp=1); // 상하축 최대 속도 지정
    void z(double value, int wait=2); // 상하축 제어
    void z_test(); // 상하축 테스트
    void wait(int ms); // OMS 작업 기다리기

    void gripper(double value, int wait=2); // 그리퍼 제어
    void gripper_turn(double value, int wait=2); // 그리퍼 회전축 제어
    void servo_spin(int servo_idx, int dir);

    void setting_servo(double value, int wait=2); // 그리퍼 제어

    void servo_agl(int servo_idx, double angle, int wait=2);
    void set_rol(int servo_idx, double angle, int wait=2);
    void screw_spin(int servo_idx, int delay_time);
    void servo_stop(int servo_idx);

    void limit_Shutdown(int servo_idx, int limit_idx); // 리밋스위치 클릭시 서보모터 정지
    
    void open_pallete(int time); //팔레트 놓기기
    
    void up_grip(int time); //팔레트 올리기
    void down_grip(int time); //팔레트 내리기

    void eat_ball(int number, int speed = 300); //스크류 앞 그리퍼 동작
    void time_control_servo(int servo_idx, double dir, int time, int same = 2); //시간으로 서보모터 제어하기

    void run_dc_tick(double speed, double target_tick);

    void homing_rail();
}