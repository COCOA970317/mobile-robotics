#pragma once
#include <vector>

class Move{
private:
    static clock_t start_time; // 함수 시작 시간 저장
    static int filter_value, acc_value, accw_value, gyro_ch; // filter_value: 필터 값 저장, acc_value: 가속 값 저장, accw_value: 회전 가속 값 저장, gyro_ch: 자이로 센서 채널 지정
    static int t_ga, t_gw, f_max; // t_ga: 시간 가속, t_gw: 시간 회전 가속, f_max: 필터 연산 변수
    static double speed, wspeed, acc, accw, Ga, Gw, stop_dis, wstop_dis; // speed: 속도, wspeed: 회전 속도, acc: 가속세기, accw: 회전가속세기, Ga: 가속
    static double ve_l[3], ve_d[3], ve_x[3], ve_y[3]; // x,y -> ve_l[3]: 거리, ve_d[3]: 각도 / 거리, 각도 -> ve_x[3]: x, ve_y[3]: y
    static double rx, ry, rw; // 이전 x, y, w 데이터 저장
    static double local_en[3], global_en[3][3]; // 상대좌표, 절대좌표

    static Move instance;

    static int prev_filter_time_;
    static double current_velocity[3];

    static void Accelation(); // 가속 함수
    static void reset(int sp, int wsp, int gg, int filter); // 초기화 함수
    static void VT(double f_agl, double len, int ch); // 각도와 거리를 x, y로 변환
    static void len_VT(double x, double y, int ch); // x, y를 각도와 거리로 변환
    static int get_mode(int agl, int &num); // 모드 세팅 함수
    static int Mode(double len, int sp, int gg, int mode, int num, int value, int mm); // 탈출 조건 확인 및 감속 함수
    static void end(); // 이동 함수 끝날 때 호출하는 함수

    static void TimeBasedFilter(double target_x, double target_y, double target_w);

public:
    struct WayPoint {
        double x;
        double y;
        double w;
    };
    static void Waypoint_Drive(std::vector<WayPoint> path, int sp, int radius);

    static double filter_gain; // 필터 게인 값

    static void hold(); // 로봇 동작 중 일시정지
    static void stop(); // 정지 함수
    static void NH(double x, double y, double w);
    static void HD(double x, double y, double w, int filter);

    static double get_agl(int x, int y); // x, y 값에 따른 agl 반환
    static double get_len(int x, int y); // x, y 값에 따른 len 반환

    // Set Accelation & Decelation
    static Move set_acc(int ms); // 가속 시간 설정
    static void dec(int ms); // 감속 함수 (ms 시간 동안 감속)
    static Move set_filter(int filter=50); // filter값 설정
    static Move filter_off(); // filter 끄기
    static Move set_gyro(int ch, int heading=0); // 자이로 채널 설정
    static Move gyro_off(); // gyro 끄기
    

    // Basic Move

    /*
        num 인자에 사용되는 조건들
        LF=1, // Left Front (현재 Ultra Left)
        FL=2, // Front Left (현재 PSD Left)
        FR=3, // Front Right (현재 PSD Right)
        RF=4, // Right Front (현재 Ultra Right)

        COB=8, // 라인 감지
        COB_LOST=9, // 라인 사라짐
        DIS=10, // 거리
    */

    // gg: 십의자리 가속여부, 일의자리 감속여부
    // ex) 11(가감속 둘다), 10(가속만), 01(감속만), 00(가감속 없음)
    
    static void gyro_reset();
    static void position(int set); // 절대 좌표 초기화 및 모든 좌표 연산
    // ch: 채널 번호 지정, sp: 속도, wsp: 회전속도, gg: 가감속 여부, x: 
    static void global(int ch, int sp=400, int wsp=0, int gg=11, int x=0, int y=0, int w=0, int turn_dir=0); // 절대좌표 이동

    // agl: 진행방향, sp: 속도, gg: 가감속 여부
    static void move(int agl, int sp, int gg, int x, int y); // x y 이동 함수 (mm단위)
    static void turn(int sp, int gg, int w); // 회전함수 (degree)
    static void Curve(int sp, int turn_sp, int target_angle);
    static void holonomic(int agl, int sp, int gg, int x, int y, int w); // 홀로노믹 x y w 이동 함수 (mm단위)

    static void check_turn(int ch, int target_agl);

    // Sensor Move
    // value: 앞벽 붙을 값, sub_num: 서브 보정 종류, sub_value: 서브 보정 값, ratio: 센서 보정 비율
    static void cp_front(int target_dist, int time_out);
    static void cp(int value, int sub_num=0, int sub_value=0, double ratio=0); // 보정 함수
    static void align_parallel(int value, double ratio=0); // 순수 회전으로 벽과 평행 맞추기

    // agl: 진행방향, sp: 속도, gg: 가감속 여부, num: 센서 종류, value: 붙거나 떨어질 값, mm: 무시거리
    static void agl_sensor(int agl, int sp, int gg, int num, int value, int mm=0, int cnt = 1);

    // ta: 벽타는 값, sp: 속도, gg: 가감속 여부, num: 센서 종류, value: 목표 값, mm: 무시거리
    static void left_wall_move(int ta, int sp, int gg, int num, int value, int mm=0); // y 방향 이동, 왼쪽 벽타기
    static void right_wall_move(int ta, int sp, int gg, int num, int value, int mm=0); // y 방향 이동, 오른쪽 벽타기

    static void FL_wall_move(int ta, int sp, int gg, int num, int value, int mm=0); // y 방향 이동, FL(앞-왼) 센서 사용
    static void FR_wall_move(int ta, int sp, int gg, int num, int value, int mm=0); // y 방향 이동, FR(앞-오) 센서 사용
    
    static void front_wall_move(int ta, int sp, int gg, int num, int value, int mm=0); // x 방향 이동, 앞 벽타기
    static void back_wall_move(int ta, int sp, int gg, int num, int value, int mm=0); // x 방향 이동, 뒷벽 타기
    static void front_wall_dist(int ta, int sp, int gg, int mm=0);

    static void agl_sensor_center(int f_agl, int sp, int gg, int num, int value, int cs = 600, int mm = 0, int cnt=0);

    static void center_FB_move(int sp, int gg, int mm);

    // agl: 진행방향, sp: 속도, gg: 가감속 여부, n: n번째 라인, value: 감속거리, mm: 무시거리
    static void line_find(int agl, int sp, int gg, int n=1, int value=0, int mm=0); // n번째 라인 찾는 함수
    static void line_lost(int agl, int sp, int gg, int value, int mm=0); // 라인 사라지면 탈출
    static void line(int sp, int gg, int num, int value, int mm=0); // 라인타는 함수
    static int move_and_count_lines(int agl, int sp, int gg, int x, int y);
    static double move_length(int agl, int sp, int gg, int num, int value, int mm=0, int cnt = 1);

    static void cam_tracking(int color_num, int target_x, int target_y);
};