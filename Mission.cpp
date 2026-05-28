#include "Mission.h"
#include "ShuffleBoard.h"

// ---------------------------------------------------------
// Global Variables & Arrays
// ---------------------------------------------------------
int turnval = 2;
int R=187-16;
int rc = 0, oc = 0, yc = 0, gc = 0;
int FRnum = 0, FLnum = 0, RFnum = 0, LFnum = 0, BSnum = 0;
float Favg = 0.0, favg = 0.0;

#define STOP_CHECK if(::M_cnt == 0 || ::ems) return;
using namespace std;


int pallete_agl = -90;
int gripper_agl = 0;

int mspd = 500;
int spd = 500;
int lspd = 200;
int wspd= 120;
int linespd = 100;

int eat_time = 745;
int up_time = 0; 
int down_time = 13000;
int catch_time = 0;

int ud_grip = 2;   // 0: 내려갈 때, 300: 올라갈 때
int grap_grip = 1; // 300: 내려갈 때, 0: 올라갈 때
int rol = 0;
int screw = 3;

int current_z_time = 0; //그리퍼 타이밍 계산

int x=52;
int rol_list[3] = {18+x, -2+x, -17+x};
int carry_list[3] = {0, 405, 343};
int complete_root = 0;
int complete = 0;

int floor_num = 0;

int now_center_place = 0;
int now_baeSongGi_place = 0;
int complete_root_cnt = 0;
int want_Center_place = 0;

int ball_cnt[6] = {4+2, 3+2, 3+2, 2+2, 3, 3+2};
char carrier_pattern[6][3][4] = {
    // Pattern 1
    { "xox",
      "xox",
      "xxo" },
    // Pattern 2
    { "oxo",
      "xox", 
      "xxo" },
    // Pattern 3
    { "ooo",
      "xxx", 
      "xxx" },
    // Pattern 4
    { "oxx",
      "xxx", 
      "oxo" }, 
    // Pattern 5
    { "xxo",
      "xox",
      "oxx" },
    // Pattern 6
    { "xox",
      "xox",
      "xxx" }
};

int moveRoot[6] = {2, 5, 4, 6, 1, 3}; //배송지 이동순서서
int center_pallete_cnt[4] = {0, 2, 2, 2}; //센터에 남은 팔레트 갯수
int pattern_num = 0; //패턴 번호

// ---------------------------------------------------------
// Sensor & State Functions
// ---------------------------------------------------------
void f_color() {
    ShuffleBoard::Multi();
    rc = ShuffleBoard::Get_Count_Red();
    yc = ShuffleBoard::Get_Count_Yellow();
    oc = ShuffleBoard::Get_Count_Orange();
    gc = ShuffleBoard::Get_Count_Green();
}

void mission_start() {
    ::led_r = 0;
    ::led_g = 1;
}

void mission_end() {
    ::led_g = 0;
    ::led_r = 1;
}

// ---------------------------------------------------------
// OMS / Servo / Gripper Control Functions
// ---------------------------------------------------------

const int TIME_TOP = 0;
const int TIME_FLOOR_2 = 2000-1670;
const int TIME_FLOOR_1 = 12500-10350;

void smart_floor(int target_floor) {
    int target_time = 0;

    if(target_floor == 0) {
        current_z_time = TIME_TOP;
        return;
    }
    else if (target_floor == 2) target_time = TIME_FLOOR_2;
    else if (target_floor == 1) target_time = TIME_FLOOR_1;

    int diff = target_time - current_z_time;

    OMS::up_grip(2);
    if (diff > 0) {
        OMS::time_control_servo(ud_grip, 0, diff); // 내리기
    } 
    else if (diff < 0) {
        OMS::time_control_servo(ud_grip, 300, -diff); // 올리기
    }
    current_z_time = target_time; 
}

void chrlghk() {
    OMS::eat_ball(1, 165);
}

void save_ball() {
    OMS::time_control_servo(3, 300, 745 * 10, 0);
    OMS::eat_ball(1, 180);
}

void classify_ball() {
    OMS::servo_agl(4, 165-65);
    delay(500);
    
    for(int i = 0; i < 3; i++){
        OMS::run_dc_tick(-0.4, carry_list[i]);
        delay(500);  // 슬라이드 완전히 멈춘 후 대기
        
        for(int j = 0; j < 3; j++){
            if(carrier_pattern[pattern_num-1][i][j] == 'o'){
                OMS::set_rol(0, rol_list[j], 0);
                OMS::servo_agl(4, 245);
                delay(1000);
                OMS::servo_agl(4, 165-65);
                delay(2300);
            }
        }
    }
    OMS::homing_rail();
}
void tight_gripper() {
    OMS::time_control_servo(grap_grip, 300, 3000);
}

void set_floor2() {
    OMS::open_pallete(0);
    OMS::up_grip(2);
    smart_floor(0);
    OMS::time_control_servo(ud_grip, 0, 2000);
}

void set_floor1() {
    OMS::time_control_servo(ud_grip, 0, 12000);
}

void get_floor1() {
    OMS::open_pallete(0);
    OMS::up_grip(2);
    smart_floor(0);
    OMS::time_control_servo(ud_grip, 0, 12000);
}

void start_setting() {
    chrlghk();
    OMS::set_rol(0, 18+x, 0); 
    OMS::servo_agl(4, 165-65, 0);
    OMS::open_pallete(0);
    OMS::up_grip(0);
    smart_floor(0);
}

// ---------------------------------------------------------
// Complex Movement & Logistics Functions
// ---------------------------------------------------------
void get_pallete(int agl) {
    Move::line_find(agl, linespd, 11);
    Move::line_lost(-90, linespd, 10, 0);
    Move::move(-90, linespd, 01, 0, 22);
    Move::stop();

    std::thread(tight_gripper).detach();
    OMS::up_grip(2);
    smart_floor(0);
    delay(2000);
}

void put_pallete() {
    OMS::open_pallete(0);
    smart_floor(1);
    delay(1000);
    Move::move(90, lspd, 11, 0, 200);
}

void eat_railball_3(int cnt_left, int cnt_right) {
    Move::line_find(180, linespd, 01);
    Move::line_lost(90, 200, 11, 0);

    Move::turn(wspd, 11, -88);
    Move::stop();
    Move::move(-90, linespd, 11, 0, 114);
    Move::move(0, lspd, 11, 0, 170);
    Move::stop();

    OMS::eat_ball(cnt_left);

    Move::move(180, 200, 11, 0, 100);
    std::thread(chrlghk).detach();
    Move::move(90, linespd, 11, 0, 69);
    Move::move(0, lspd, 11, 0, 130);
    Move::stop();

    OMS::eat_ball(cnt_right);
}

void eat_railball_2(int cnt_left, int cnt_right) {
    Move::line_find(180, linespd, 01);
    Move::line_lost(90, 200, 11, 0);

    Move::turn(wspd, 11, -88);
    Move::stop();
    Move::move(-90, linespd, 11, 0, 113);
    Move::move(0, lspd, 11, 0, 160);
    Move::stop();

    OMS::eat_ball(cnt_left);

    Move::move(180, 200, 11, 0, 100);
    std::thread(chrlghk).detach();
    Move::move(90, linespd, 11, 0, 76);
    Move::move(0, lspd, 11, 0, 130);
    Move::stop();

    OMS::eat_ball(cnt_right);
}

void eat_railball_1(int cnt_left, int cnt_right) {
    Move::line_find(180, linespd, 01);
    Move::line_lost(90, 200, 11, 0);

    Move::turn(wspd, 11, -88);
    Move::stop();
    Move::move(-90, linespd, 11, 0, 102);
    Move::move(0, lspd, 11, 0, 160);
    Move::stop();

    OMS::eat_ball(cnt_left);

    Move::move(180, 200, 11, 0, 100);
    std::thread(chrlghk).detach();
    Move::move(90, linespd, 11, 0, 68);
    Move::move(0, lspd, 11, 0, 140);
    Move::stop();

    OMS::eat_ball(cnt_right);
}

const int dist_1[3] = {114, 113+3, 103-5}; 
const int dist_2[3] = {69, 73, 70};

void eat_railball(int target, int cnt_left, int cnt_right) {
    int idx = 3 - target; 

    Move::line_find(180, linespd, 01);
    Move::line_lost(90, 200, 11, 0);

    if(target == 3 || target == 1){
        Move::turn(wspd, 11, -90+turnval-1.5);
    }else{
        Move::turn(wspd, 11, -90+turnval);
    }
    Move::stop();
    Move::move(-90, linespd, 11, 0, dist_1[idx]);
    if(target == 3 || target == 1){
        Move::move(0, lspd, 11, 0, 170+40); 
    }else{
        Move::move(0, lspd, 11, 0, 170+20); 
    }
    Move::stop();

    OMS::eat_ball(cnt_left);

    Move::move(180, 200, 11, 0, 100);
    std::thread(chrlghk).detach();
    Move::move(90, linespd, 11, 0, dist_2[idx]);
    Move::move(0, lspd, 11, 0, 130-30);
    Move::stop();

    OMS::eat_ball(cnt_right);
}

// ---------------------------------------------------------
// Section Movements (Point to Point)
// ---------------------------------------------------------
void C1B2(){
    Move::move(90, mspd, 10, 0, 50);
    Move::move(0, mspd, 00, -50, 330);
    Move::move(-90, mspd, 00, 0, 400);
    Move::holonomic(180, mspd, 01, 600, 400, 180);
    Move::stop();
    Move::move(0, 100, 10, 0, 20);
    Move::cp(250);
    Move::turn(linespd, 11, 90);
    Move::stop();
    Move::line_find(180, linespd, 11);
    Move::line_lost(-90, 60, 11, 0);
    Move::stop();
}

void B2C2() {
    Move::move(90, spd, 10, 0, 150);
    Move::holonomic(90, spd, 01, 400, 700, 90);
    Move::stop();
    Move::cp(220);
    Move::stop();
    Move::move(-90, lspd, 11, 0, 170);
    Move::stop();
}

void C2B5() {
    Move::move(90, lspd, 10, 0, 150);
    Move::holonomic(180, spd, 00, -50, 520, -45);
    Move::holonomic(-45, spd, 01, -200, 1400, -45);
    Move::stop();
    Move::move(0, 100, 10, 0, 20);
    Move::cp(250);
    Move::turn(linespd, 11, 90);
    Move::stop();
    Move::line_find(0, linespd, 11);
    Move::line_lost(-90, 60, 11, 0);
    Move::stop();
}

void B5C3() {
    Move::move(0, spd, 11, 0, 600);
    Move::cp(350);
    Move::stop();
    Move::turn(wspd, 11, -178);
    Move::move(-90, lspd, 10, 0, 50);
}

void C3B6() {
    Move::move(90, lspd, 10, 0, 150);
    Move::holonomic(90, spd, 01, 200, 500, 90);
    Move::stop();

    Move::cp(100);
    Move::front_wall_move(90, -spd, 11, LF, 280);

    Move::line_find(180, linespd, 11);
    Move::line_lost(-90, 60, 11, 0);
    Move::stop();
}

void C_TO_C(int Center1, int Center2){
    int route_id = (Center1 * 10) + Center2;

    switch(route_id) {
        //Center1->Center[n]
        case 12:
            break;

        case 13:

            break;

        //Center2->Center[n]
        case 21:
            Move::holonomic(0, spd, 11, -550, -1000, 90);
            Move::agl_sensor(90, lspd, 10, RF, 120);
            Move::agl_sensor(0, lspd, 01, FL, 100);
            Move::cp(100, RF, 120);
            Move::move(-90, lspd, 10, -450, 300);
            Move::stop();
            break;

        case 23:

            break;

        //Center3->Center[n]
        case 31:
            
            break;

        case 32:
            Move::move(90, spd, 00, 0, 750);
            Move::move(0, spd, 00, 0, 750);
            Move::move(0, spd, 00, -350, 550);
            Move::turn(wspd, 01, -90+turnval);
            Move::stop();
            Move::agl_sensor(0, spd, 11, FLR, 290);
            Move::cp(290);

            Move::move(-90, lspd, 10, 0, 90);
            break;
    }
    now_center_place = Center2;
    ShuffleBoard::nt_now_center.SetDouble(now_center_place);
}

void C_TO_B(int center, int address) {
    int route_id = (center * 10) + address;

    switch(route_id) {
        //Center1 -> BaeSongGi[n]
        case 11: //qkrrjsdn
            Move::agl_sensor(0, spd, 10, FLR, 400);
            Move::cp(250);
            Move::stop();
            Move::turn(wspd, 11, 180-turnval);
            Move::back_wall_move(110, -(lspd+100), 10, LF, 350);
            Move::line_find(0, linespd, 01);
            Move::line_lost(-90, 60, 11, 0);
            Move::stop();
            break;

        case 12: //qkrrjsdn
            //Move::move(90, mspd, 10, 0, 50);
            Move::move(0, mspd, 10, 0, 330);
            Move::move(-90, mspd, 00, 0, 400-50-50);
            Move::holonomic(180, mspd, 01, 700, 400, 180);
            Move::stop();
            Move::move(0, 100, 10, 40, 20);
            Move::cp(250);
            Move::turn(linespd, 11, 90);
            Move::stop();
            Move::line_find(180, linespd, 11);
            Move::line_lost(-90, 60, 11, 0);
            Move::stop();
            break;

        case 13:

            break;

        case 14:

            break;
        
        case 15:

            break;

        case 16:

            break;

        //Center2 -> BaeSongGi[n]
        case 21:
            Move::move(0, spd, 11, 300, -100);
            Move::cp(100);
            Move::turn(wspd, 11, -180);
            Move::stop();
            Move::move(0, spd, 10, -400, 0);
            Move::line_find(0, 100, 01, 1);
            Move::line_lost(-90, 100, 11, 0);
            Move::stop();
            break;

        case 22:
            Move::move(0, spd, 11, 250, -70);
            Move::cp(100);
            Move::turn(wspd, 11, -90);
            Move::move(0, spd, 10, -230, 0);
            Move::move(0, spd, 00, -680, 600);
            Move::move(0, spd, 00, -200, 0);
            Move::line_find(180, 100, 01, 1);
            Move::line_lost(-90, 100, 11, 0);
            Move::stop();
            break;

        case 23:
            Move::move(0, spd, 11, 250, -70);
            Move::cp(100);
            Move::turn(wspd, 11, -90);
            Move::move(0, spd, 10, -230, 0);
            Move::move(0, spd, 00, -680, 600);
            Move::move(0, spd, 00, -200, 0);
            Move::line_find(0, 100, 01, 1);
            Move::line_lost(-90, 100, 11, 0);
            Move::stop();
            break;

        case 24:
            Move::move(0, spd, 11, 300, -70);
            Move::cp(100);

            Move::move(0, spd, 10, 0, -500);
            Move::move(0, spd, 00, -200, 0);
            Move::move(0, spd, 00, 0, -200);
            Move::move(0, spd, 00, 0, -200);
            Move::move(0, spd, 00, -1000, 0);
            Move::line_find(180, 100, 01, 1);
            Move::line_lost(-90, 100, 11, 0);
            Move::stop();
            break;

        case 25: //qkrrjsdn
            Move::move(90, lspd, 10, 0, 150);
            Move::holonomic(180, spd, 00, -50, 550, -45);
            Move::holonomic(-45, spd, 01, -250, 1400, -45);
            Move::stop();
            Move::move(0, 100, 10, -150, 20);
            Move::cp(250);
            Move::turn(linespd, 11, 90-turnval);

            Move::stop();
            Move::line_find(0, linespd, 11);
            Move::line_lost(-90, 60, 11, 0);
            Move::stop();
            break;

        case 26:
            Move::move(0, spd, 11, 300, 0);
            Move::cp(100);
            Move::turn(wspd, 11, -90);
            Move::move(0, spd, 10, 0, -500);
            Move::move(0, spd, 00, -400, 600);
            Move::move(0, spd, 00, 1000, 1300);
            Move::front_wall_move(150, -spd, 01, LF, 300);
            Move::cp(100, LF, 300);
            Move::line_find(180, 100, 01, 1);
            Move::line_lost(-90, 100, 11, 0);
            Move::stop();
            break;

        //Center3 -> BaeSongGi[n]
        case 31: //rlawodn
            Move::move(0, spd, 10, 0, 500);
            Move::move(0, spd, 00, -400, 0);
            Move::move(0, spd, 00, -1200, -1000);
            Move::back_wall_move(200, -spd, 01, LF, 350);
            Move::line_find(0, 100, 11, 1);
            Move::line_lost(-90, 60, 11, 0);
            Move::stop();
            break;

        case 32:

            break;

        case 33: //rlawodn
            Move::move(0, spd, 10, 0, 550);
            Move::move(0, spd, 00, -300, 0);
            Move::holonomic(0, spd, 01, -200, 0, 90);
            Move::line_find(180, 100, 11, 1);
            Move::line_lost(-90, 100, 11, 0);
            Move::stop();
            break;

        case 34: //rlawodn
            Move::move(90, lspd, 10, 0, 50);
            Move::holonomic(0, spd, 01, 100, 650, 180-turnval);
            Move::move(-90,spd,11,0,60);
            Move::line_find(-180, 100, 01, 1);
            Move::line_lost(-90, 100, 11, 0);
            Move::stop();
            break;

        case 35: //rlawodn
            Move::holonomic(0, spd, 10, 300, 400, 180);
            Move::line_find(-180, 100, 11, 1);
            Move::line_lost(-90, 100, 11, 0);
            Move::stop();
            break;

        case 36: //qkrrjsdn
            Move::move(90, lspd, 10, 0, 150);
            Move::holonomic(90, spd, 01, 200, 500, 90);
            Move::stop();
            Move::move(0,spd,11,0,100);
            Move::cp(100);
            Move::front_wall_move(90, -spd, 11, LF, 280,400);
            Move::line_find(180, linespd, 11);
            Move::line_lost(-90, 60, 11, 0);
            Move::stop();
            break;

        default:
            break;
    }
    now_baeSongGi_place = address;
    ShuffleBoard::nt_now_BaeSongGi.SetDouble(now_baeSongGi_place);
}

void B_TO_C(int address, int center) {
    int route_id = (address * 10) + center;

    switch(route_id) {
        //BaeSongGi1 -> Center[n]
        case 11:

            break;

        case 12: //
            Move::move(0, spd, 10, 500, 0);
            Move::turn(wspd, 01, 180);
            Move::cp(130);
            Move::move(0, spd, 11, -30, 0);
            break;

        case 13:

            break;

        //BaeSongGi2 -> Center[n]
        case 21:

            break;

        case 22: //qkrrjsdn
            Move::move(90, spd, 10, 0, 150);
            Move::holonomic(90, spd, 01, 400, 700, 90);
            Move::stop();
            Move::cp(220);
            Move::stop();
            Move::move(-90, lspd, 11, 0, 230);
            Move::stop();
            break;

        case 23:

            break;

        //BaeSongGi3 -> Center[n]
        case 31:

            break;

        case 32:

            break;

        case 33:
                break;

        //BaeSongGi4 -> Center[n]
        case 41:

            break;

        case 42:

            break;

        case 43:
            Move::turn(wspd, 01, 178);
            Move::stop();
            Move::cp(300);
            Move::move(180, spd, 11, 50, 800);
            Move::stop();
            break;

        //BaeSongGi5 -> Center[n]
        case 51:

            break;

        case 52:
        
            break;

        case 53: //qkrrjsdn
            Move::move(0, spd, 11, 0, 600);
            Move::cp(350);
            Move::stop();
            Move::turn(wspd, 11, -178);
            Move::move(-90, lspd, 11, 0, 40);
            Move::stop();
            break;

        //BaeSongGi6 -> Center[n]
        case 61: //
            Move::move(0, spd, 10, 150, 0);
            Move::front_wall_move(110, spd, 00, RF, 300);
            Move::turn(wspd, 01, 90);
            Move::cp(100, LF, 100);

            Move::move(0, spd, 10, 300, 0);
            Move::move(0, spd, 00, 1000, -800);
            Move::move(90, spd, 00, 0, 750);
            Move::move(0, spd, 00, 300, 400);
            Move::move(0, spd, 01, 700, -550);
            Move::cp(100, RF, 100);
            Move::move(0, spd, 11, -300, -150);
            Move::stop();
            break;

        case 62:

            break;

        case 63:

            break;

        default:
            break;
    }
    now_center_place = center;
    ShuffleBoard::nt_now_center.SetDouble(now_center_place);
}

void H_to_C(int center) {
    switch(center) {
        case 1:

            break;

        case 2:

            break;

        case 3:

            break;

        case 4:

            break;

        case 5:

            break;

        case 6:

            break;
    }
    now_center_place = center;
    ShuffleBoard::nt_now_center.SetDouble(now_center_place);
}

void ready_to_eat_pallete(int center) {
    switch(center) {
        case 1:
            Move::cp(100, RF, 120);
            Move::move(-90, lspd, 11, -150, 300);
            Move::stop();
            break;

        case 2:

            break;

        case 3:

            break;
    }
}
// ---------------------------------------------------------












// Main Mission Functions (M1 - M20)












// ---------------------------------------------------------
// void M1(){
//     mission_start();
//     std::thread(start_setting).detach();
//     Move::turn(120, 11, -90);
//     Move::stop();

//     Move::cp(100, LF, 120);
//     Move::turn(120,11, 178);

//     Move::back_wall_move(200, -400, 10, 0, 0, 1100);
//     eat_railball_3(ball_cnt[0], ball_cnt[1]);
    
//     Move::move(180, 400, 10, 0, 100);
//     std::thread(save_ball).detach();

//     Move::move(90, spd, 00, 0, 750);
//     Move::move(0, spd, 00, 0, 750);
//     Move::move(0, spd, 00, -300, 550);
//     Move::turn(wspd, 01, -87);
//     Move::stop();
//     Move::agl_sensor(0, spd, 11, FLR, 290);
//     Move::cp(290);

//     Move::move(-90, lspd, 10, 0, 90);
//     eat_railball_2(ball_cnt[2], ball_cnt[3]);
    
//     Move::move(180, 400, 10, 0, 100);
//     std::thread move_eat(save_ball);

//     Move::holonomic(0, spd, 11, -650, -950, -90);
//     Move::cp(250);

//     if (move_eat.joinable()) {
//         move_eat.join();
//     }

//     Move::turn(wspd, 11, 178);
//     Move::move(-90, lspd, 10, 0, 140);
//     eat_railball_1(ball_cnt[4], ball_cnt[5]);
    
//     Move::move(180, spd, 11, 0, 100);
//     std::thread(save_ball).detach();

//     Move::move(0, spd, 10, 550, -350);
//     Move::turn(wspd, 01, 88);
//     Move::stop();
//     Move::cp(100, RF, 120);
//     std::thread sub_task(set_floor2);
//     Move::move(-90, lspd, 11, -150, 300);
//     Move::stop();
//     if (sub_task.joinable()) {
//         sub_task.join();
//     }

//     get_pallete(180); 
    
//     std::thread sub_task2(C1B2);
    
//     put_pallete();
    
//     std::thread sub_task3(B2C2);
//     set_floor2();
//     if (sub_task3.joinable()) {
//         sub_task3.join();
//     }
//     get_pallete(0);

//     std::thread(C2B5).detach();

//     put_pallete();

//     mission_end();
// }

// void M2(){
//     mission_start();
//     std::thread(start_setting).detach();

//     Move::cp(100, LF, 120);
//     Move::turn(120,11, 178);

//     Move::back_wall_move(200, -400, 10, 0, 0, 1100);
//     eat_railball(3, ball_cnt[0], ball_cnt[1]);

//     Move::move(180, 400, 10, 0, 100);
//     std::thread(save_ball).detach();

//     C_TO_C(3,2);
//     eat_railball(2, ball_cnt[2], ball_cnt[3]);

//     Move::move(180, 400, 10, 0, 100);
//     std::thread move_eat(save_ball);
//     C_TO_C(2,1);
//     if (move_eat.joinable()) {
//         move_eat.join();
//     }

//     eat_railball(1, ball_cnt[4], ball_cnt[5]);

//     Move::move(180, spd, 11, 0, 100);
//     std::thread(save_ball).detach();

//     //공 먹기 끝

//     Move::move(0, spd, 10, 450, -250);
//     std::thread sub_task(smart_floor, 2);
//     Move::turn(wspd, 01, 88);
//     Move::stop();
//     ready_to_eat_pallete(1);
//     if(sub_task.joinable()){
//         sub_task.join();
//     }


//     get_pallete(180);

//     std::thread move_task1(C_TO_B, 1, 2);
//     pattern_num = 2;
//     classify_ball();
//     if(move_task1.joinable()) move_task1.join();

//     put_pallete();
    
//     //1번 완료
//     std::thread return_task1(B_TO_C, now_baeSongGi_place, 2);
//     if(center_pallete_cnt[2] == 2) {
//         smart_floor(2);
//     }
//     else if(center_pallete_cnt[2] == 1) {
//         smart_floor(1);
//     }

//     if(return_task1.joinable()) return_task1.join();

//     get_pallete(0);

//     std::thread move_task2(C_TO_B, now_center_place, moveRoot[complete_root_cnt]);
//     pattern_num = moveRoot[complete_root_cnt];
//     classify_ball();
//     if(move_task2.joinable()) move_task2.join();

//     std::thread return_task2(B_TO_C, now_baeSongGi_place, 3);
//     if(center_pallete_cnt[3] == 2) {
//         smart_floor(2);
//     }
//     else if(center_pallete_cnt[3] == 1) {
//         smart_floor(1);
//     }
//     if(return_task2.joinable()) return_task2.join();

//     //2번 station
//     get_pallete(180);
// }   

void M1() {
    ShuffleBoard::nt_delivery_x.SetString("");
    ShuffleBoard::nt_delivery_y.SetString("");
    ShuffleBoard::nt_mode.SetDouble(10); //바코드 읽는 모드
    double xs=0.0;
    double ys=0.0;
    while (!::ems && xs==0.0 && ys == 0.0) {
        xs = ShuffleBoard::nt_delivery_x.GetDouble(0.0);
        ys = ShuffleBoard::nt_delivery_y.GetDouble(0.0);
        delay(5000);
    }
    ShuffleBoard::nt_mode.SetDouble(0);
    int ix = (int) xs;
    int iy = (int) ys;
    Move::move(0,450,11,ix,ys);
}

void M2() {
    Move::move(0,450,11,0,400);
    Move::turn(450,11,180);
    Move::move(0,450,11,0,400);
}

void M3(){
    Move::move(180, 400, 10, 0, 100);
    std::thread move_eat(save_ball);

    C_TO_C(2, 1);
    eat_railball(1, ball_cnt[4], ball_cnt[5]);
}

void M4(){
    center_pallete_cnt[1] = 2;
    center_pallete_cnt[2] = 2;
    center_pallete_cnt[3] = 2;
    mission_start();
    std::thread(start_setting).detach();

    Move::cp(100, LF, 120);

    Move::front_wall_move(300, 400, 10, 0, 0, 400);
    Move::move(90,spd,00,550,1000);
    Move::move(90,spd,00,0,650);
    Move::move(45,spd,00,0,850);
    Move::cp(150);
    Move::move(-90,spd,00,0,200);
    eat_railball(2, ball_cnt[0], ball_cnt[1]);

    Move::move(180, 400, 10, 0, 100);
    std::thread(save_ball).detach();
    Move::move(-90,spd,00,0,180);
    Move::move(-30,spd,00,0,700);
    Move::move(0,spd,00,0,680);
    Move::holonomic(90,spd,01,0,800,90);
    Move::stop();
    Move::cp(200);
    Move::stop();
    Move::turn(wspd,11,180-turnval);
    Move::stop();
    eat_railball(3, ball_cnt[2], ball_cnt[3]);

    Move::move(180, 400, 10, 0, 100);
    std::thread(save_ball).detach();
    Move::move(90,spd,00,0,700);
    Move::move(0,spd,00,0,870);
    Move::holonomic(-47,spd,01,0,850,-90);
    Move::stop();
    Move::move(90+45,spd,10,0,450);
    Move::move(90,spd,00,30,300+350);
    Move::cp(100, RF, 120);
    Move::move(-90, lspd, 10, -450, 300);
    Move::stop();
    
    eat_railball(1, ball_cnt[4], ball_cnt[5]);

    Move::move(180, spd, 11, 0, 100);
    std::thread(save_ball).detach();

    start_setting();
    Move::move(0, spd, 10, 450, -210);
    if(center_pallete_cnt[2] == 2){
        floor_num = 2;
        center_pallete_cnt[2] -= 1;
    }
    else if(center_pallete_cnt[2] == 1) {
        floor_num = 1;
        center_pallete_cnt[2] -= 1;
    }
    smart_floor(2);
    Move::turn(wspd, 01, 88);
    Move::stop();
    ready_to_eat_pallete(1);
//1번 시작
    get_pallete(180);
    center_pallete_cnt[3] -= 1;
    C_TO_B(1,2);
    pattern_num = 2;
    classify_ball();

    put_pallete();
//1번 완료
//2번 시작 준비
    std::thread(start_setting).detach();
    if(center_pallete_cnt[2] == 2) {
        floor_num = 2;
        center_pallete_cnt[2] -= 1;
    }
    else if(center_pallete_cnt[2] == 1) {
        floor_num = 1;
        center_pallete_cnt[2] -= 1;
    }
    B_TO_C(2,2);
    std::thread return_task1(smart_floor, 2);
    if(return_task1.joinable()) return_task1.join();
//2번 시작
    get_pallete(0);

    C_TO_B(2,5);
    pattern_num = 5;
    classify_ball();

    put_pallete();
//2번 끝
//3번 시작 준비
    std::thread return_task2(start_setting);
    if(center_pallete_cnt[3] == 2) {
        floor_num = 2;
        center_pallete_cnt[3] -= 1;
    }
    else if(center_pallete_cnt[3] == 1) {
        floor_num = 1;
        center_pallete_cnt[3] -= 1;
    }
    
    B_TO_C(5,3);
    Move::turn(120,11,180-turnval);
    Move::cp(260);
    Move::turn(120,11,180-turnval);
    Move::stop();
    if(return_task2.joinable()) return_task2.join();
    smart_floor(2);
//3번 시작
    get_pallete(0);

    C_TO_B(3,4);
    pattern_num = 4;
    classify_ball();

    put_pallete();
//3번 끝
//4번 시작 준비
    Move::line_lost(90, 60, 11, 0);
    //Move::move(90, spd, 11, 0, 150);
    std::thread return_task3(start_setting);
    if(center_pallete_cnt[3] == 2) {
        floor_num = 2;
        center_pallete_cnt[3] -= 1;
    }
    else if(center_pallete_cnt[3] == 1) {
        floor_num = 1;
        center_pallete_cnt[3] -= 1;
    }
    
    B_TO_C(4,3);
    if(return_task3.joinable()) return_task3.join();
    smart_floor(1);
    
//4번 시작
    get_pallete(180);

    C_TO_B(3,6);
    pattern_num = 6;
    classify_ball();

    put_pallete();
//4번 끝
//5번 시작 준비
    std::thread return_task4(start_setting);
    if(center_pallete_cnt[1] == 2) {
        floor_num = 2;
        center_pallete_cnt[1] -= 1;
    }
    else if(center_pallete_cnt[1] == 1) {
        floor_num = 1;
        center_pallete_cnt[1] -= 1;
    }
    
    B_TO_C(6,1);
    if(return_task4.joinable()) return_task4.join();
    smart_floor(1);
    
//5번 시작
    get_pallete(180);

    C_TO_B(1,1);
    pattern_num = 1;
    classify_ball();

    put_pallete();
//5번 끝
//6번 시작 준비
    std::thread return_task5(start_setting);
    if(center_pallete_cnt[2] == 2) {
        floor_num = 2;
        center_pallete_cnt[2] -= 1;
    }
    else if(center_pallete_cnt[2] == 1) {
        floor_num = 1;
        center_pallete_cnt[2] -= 1;
    }
    
    B_TO_C(1,2);
    if(return_task5.joinable()) return_task5.join();
    smart_floor(1);
    
    
//6번 시작
    get_pallete(0);

    C_TO_B(2,3);
    pattern_num = 3;
    classify_ball();

    put_pallete();

    
//6번 끝
//홈으로 돌아가기기
    Move::move(0, spd, 10, 150, 0);
    Move::move(0, spd, 00, 0, 200);
    Move::move(0, spd, 01, 1100, 1300);
    Move::cp(100, RF, 100);
}

void M5(){

}

void M6(){ 
    start_setting();
    smart_floor(2);
    ready_to_eat_pallete(1);
    get_pallete(180);
    center_pallete_cnt[3] -= 1;
    C_TO_B(1,2);
    pattern_num = 2;
    classify_ball();
    put_pallete();
}

void M7(){
    ::led_g = 1;
}
void M8(){
    mission_end();
}
void M9(){

}
void M10(){}
void M11(){}
void M12(){}
void M13(){}
void M14(){}
void M15(){}
void M16(){}
void M17(){}
void M18(){}
void M19(){}
void M20(){}

// ---------------------------------------------------------
// Initialization & Entry Points
// ---------------------------------------------------------
void Mission_Init(){
    cout << endl;
    cout << "********** Mission Init *********" << endl;
}

void All(){
    // void (*MISSION[21])()={0,M1,M2,M3,M4,M5,M6,M7,M8,M9,M10,M11,M12,M13,M14,M15,M16,M17,M18,M19,M20};
    // for(int i=1;i<=1;i++) (*(MISSION+i))();
}