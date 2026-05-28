#include "All.h"
#include "studica/Servo.h"
#include "studica/ServoContinuous.h"
bool task_add_flg;
double task[30][3], task_value[Constants::SERVO_CNT+3], servo_control_mode[Constants::SERVO_CNT];
int task_cnt, mode;
int prev_z = -1;

double max_speed = 1;

clock_t task_time[Constants::SERVO_CNT+3];

void _z(){
    double value, dis, sp;
    bool end = false, acc_flg = false;
    
    if(task_value[OMS_Z] != -1){
        value = task_value[OMS_Z];

        end = fabs(task_value[OMS_Z] - ::oms_z_now)<4;
        
        if(end){
            prev_z = task_value[OMS_Z];
            task_value[OMS_Z] = -1;
            value = prev_z;
        }
        if(mode==OMS_Z && (clock()-task_time[OMS_Z])/CLOCKS_PER_SEC >= Constants::TIMEOUT_SEC){
            prev_z = -1;
            task_value[OMS_Z] = -1;
            value = prev_z;
        }
        acc_flg = true;
    }
    else if(prev_z != -1) value = prev_z;
    else value = -1;

    if(value != -1){
        dis = value - ::oms_z_now;
        if(value > ::oms_z_now){ // Up
            static const double oms_z_v1[2][3] = { { 0, 2, 120}, { 0, 0.14, 1} }; 
            sp = Trans(Arr(oms_z_v1), fabs(dis)) * sign(dis);
        }
        else{ // Down
            static const double oms_z_v2[2][3] = { { 0, 3, 120}, { 0, 0.10, 1} };
            sp = Trans(Arr(oms_z_v2), fabs(dis)) * sign(dis);
        }
        if(acc_flg) sp *= Trans(0,200,0,1,(clock()-task_time[OMS_Z])/1000);
        sp = std::clamp(sp,-max_speed,max_speed);
        ::motor[3] = sp;
    }
}
void _servo(){
    double value, acc_dec, target;

    for(int i=0;i<Constants::SERVO_CNT;i++){
        target = task_value[i+3];
        if(task_value[i+3] == -1) ::servo[i] = -1;
        else if(servo_control_mode[i]==1){ // 서보 가감속
            value = ::servo[i];
            acc_dec = Trans(0,200,0.1,2.3,(clock()-task_time[i+3])/1000); // 가속 (2.6)
            acc_dec = Trans(0,27,0.1,acc_dec,fabs(value-target)); // 감속

            if(value + acc_dec < target) value += acc_dec;
            else if(value - acc_dec > target) value -= acc_dec;
            else{
                value = target;
                task_time[i+3]=0;
            }
            ::servo[i] = value;
        }
        else if(servo_control_mode[i]==2){ // slow
            const double gain = 0.75;
            value = ::servo[i];
            if(value + gain < target) value += gain;
            else if(value - gain > target) value -= gain;
            else{
                value = target;
                task_time[i+3]=0;
            }
            ::servo[i] = value;
        }
        else{
            task_time[i+3]--;
            if(task_time[i+3]<0) task_time[i+3]=0;
            ::servo[i] = target;
        }
    }
}
void _oms_wait(){
    if(task_value[WAIT]==999){
        task_value[WAIT]=-1;
        if(!OMS::isFinished()) task_value[WAIT]=999;
    }
    else if((clock()-task_time[WAIT])/1000>=task_value[WAIT]) task_value[WAIT]=-1;
}

void run_task(){
    int m;
    if(mode==0 || OMS::isFinished(mode)){
        mode=0;
        if(task_cnt>0){
            m = task[0][0];
            if(m%10>=3) servo_control_mode[m%10-3] = m/10;
            m=m%10;

            if(task[0][2]) mode = m;
            
            task_value[m] = task[0][1];

            if(m>2 && servo_control_mode[m-3]==0) task_time[m] = ceil(fabs(task_value[m]-::servo[m-3])/2.6) + 10;
            else task_time[m] = clock();

            for(int i=0;i<task_cnt-1;i++){
                task[i][0] = task[i+1][0];
                task[i][1] = task[i+1][1];
                task[i][2] = task[i+1][2];
            }
            task_cnt--;
            task_add_flg=true;
        }
    }
    _z();
    _servo();
    _oms_wait();
}
void add_task(int m, double value, int wait){
    if(::ems) return;

    if(m==OMS_Z && value==-1){
        prev_z=-1;
        return;
    }

    if(wait==0){
        if(m%10>2) servo_control_mode[m%10-3] = m/10;
        task_value[m%10] = value;
        return;
    }

    task[task_cnt][0] = m;
    task[task_cnt][1] = value;
    task[task_cnt++][2] = wait;

    if(wait==2){
        task_add_flg=false;
        while(!::ems && !task_add_flg) delay(20);
        while(!::ems && !OMS::isFinished(m%10)) delay(20);
    }

    if(wait>2) OMS::wait(wait);
}
//======================================================================
void OMS::Loop(){
    clock_t past;

    for(int i=0;i<Constants::SERVO_CNT+3;i++) task_value[i]=-1;

    while(true){
        past = clock();

        if(::ems || ::Disable){
            prev_z=-1;
            for(int i=0;i<Constants::SERVO_CNT+3;i++) task_value[i]=-1;
            task_cnt=0;
            mode=0;
        }
        else run_task();

        if(clock()-past>=10000) cout << "OMS Timeout: " << clock()-past << endl;

        loop_delay(10, past);
    }
}

void OMS::end(int wait){
    delay(20);
    if(wait==2) while(!::ems && (!OMS::isFinished() || task_cnt)) delay(20);
    else OMS::wait(999);
}
bool OMS::isFinished(int num){
    if(num==0){
        for(int i=1;i<3;i++) if(task_value[i]!=-1) return false;
        for(int i=3;i<Constants::SERVO_CNT+3;i++) if(task_time[i]!=0) return false;
    }
    else if(num<3) return task_value[num]==-1;
    else return task_time[num]==0;
    return true;
}
void OMS::set_speed(double sp){
    max_speed=sp;
}
void OMS::z(double value, int wait){
    add_task(OMS_Z, value, wait);
}
void OMS::z_test(){
    OMS::z(100);
    Move::hold();
    OMS::z(350);
    Move::hold();
    OMS::z(250);
}
void OMS::wait(int ms){
    add_task(WAIT, ms, 1);
}

void OMS::screw_spin(int servo_idx, int delay_time){
    add_task(3 + servo_idx, 300, 0);
    delay(delay_time);
    add_task(3 + servo_idx, 150, 0);
}

void OMS::servo_stop(int servo_idx){
    add_task(3+servo_idx, 150, 0);
}

//==========================================================================

void OMS::gripper(double value, int wait){
    add_task(GRIPPER, value, wait);
}

void OMS::gripper_turn(double value, int wait){
    add_task(GRIPPER_TURN, value, wait);
}


void OMS::limit_Shutdown(int servo_idx, int limit_idx) {
    while(!::ems) {
        if (LimitSW::is_pressed(limit_idx)){
            add_task(3+servo_idx, 150, 0);
            break;
        }
    }
}
//-----------------------------------------------
//number = 공 휙득 갯수
void OMS::eat_ball(int number, int speed) {
    int cnt = 0;
    int servo_idx = 3;
    add_task(3+servo_idx, speed, 0);
    for(cnt = 0; cnt < number ; ){
        if(LimitSW::is_pressed(1)){  
            std::cout << "변수에 저장된 값은 = " << cnt;
            ShuffleBoard::nt_cnt.SetDouble(cnt);
            cnt += 1;
            delay(50);
        }
        if(::ems){
            add_task(3+servo_idx, 150, 0);
            break;
        }
    }
    add_task(3+servo_idx, 150, 0);
}

//팔레트 올리기
void OMS::up_grip(int time) {  
    int cnt=0;
    if(LimitSW::is_holding(3)) return;
    int servo_idx = 2;
    add_task(3+servo_idx, 300, time);
    while(!::ems){
        cnt++;
        if(LimitSW::is_pressed(3) || cnt >= 200){
            add_task(3+servo_idx, 150, time);
            break;
        }
        delay(10);
    }
}

//팔레트 내리기
void OMS::down_grip(int time) {
    if(LimitSW::is_holding(3)) return;
    int servo_idx = 2;
    add_task(3+servo_idx, 0, time);
    while(!::ems){
        if(LimitSW::is_pressed(3)){
            add_task(3+servo_idx, 150, time);
            break;
        }
        delay(10);
    }
}

//팔레트 놓기 time = 0, 2
void OMS::open_pallete(int time) {
    if(LimitSW::is_holding(2)) return;
    int servo_idx = 1;
    add_task(3+servo_idx, 0, time);
    while(!::ems){
        if(LimitSW::is_pressed(2)) break;
        delay(10);
    }
    add_task(3+servo_idx, 150, time);
}

//servo_idx = 서보번호, dir = 회전방향(300 = 정방향 최고속도. 150 = 정지, 0 = 반대방향 최고속도), time = 서보모터 활성화 시간
void OMS::time_control_servo(int servo_idx, double dir, int time, int same) {
    add_task(3 + servo_idx, dir, same);
    delay(time);
    add_task(3 + servo_idx, 150, same);
}

void OMS::servo_spin(int servo_idx, int dir) {
    double target_value = 150; // 기본값: 정지 (중간값)

    if (dir == 1) {
        target_value = 300; // 정방향 최대 속도 (300도 지점)
    } 
    else if (dir == -1) {
        target_value = 0;   // 역방향 최대 속도 (0도 지점)
    }

    // add_task(타겟번호, 값, 대기시간)
    // 타겟번호: 3번이 서보0, 4번이 서보1 입니다. (Motor.cpp 로직 기준)
    // 대기시간 0: 즉시 실행 (가감속 없이 바로 최대속도 도달)
    add_task(3 + servo_idx, target_value, 0);
}

void OMS::servo_agl(int servo_idx, double angle, int wait) {
    // 0번 서보는 태스크 번호가 3번이므로, +3을 해줍니다.
    add_task(3 + servo_idx, Constants::BBuZZiBBuZZiBBaBBa + angle, wait);
}

void OMS::set_rol(int servo_idx, double angle, int wait) {
    add_task(3 + servo_idx, Constants::ROL_SET + angle, wait);
}

// 내부 변환 헬퍼 (파일 내부에서만 사용)
static double _to_studica(double deg) {
    return std::clamp(deg + 150.0, 0.0, 300.0);
}

static double _from_studica(double val) {
    return std::clamp(val, 0.0, 300.0) - 150.0;
}

void OMS::run_dc_tick(double speed, double target_tick) {
    if (::ems || ::Disable) return;

    task_value[OMS_Z] = -1;
    prev_z = -1;

    double start_enc = Motor::encoder[3].GetDistance();
    
    // 🚨 속도 값이 1.0을 넘더라도 안전하게 최대 1.0으로 묶어줌 (Clamp)
    double max_speed = std::clamp(std::fabs(speed), 0.0, 1.0); 
    double dir = (speed > 0) ? 1.0 : -1.0; // 앞으로 갈지 뒤로 갈지 방향 저장
    
    // ⚙️ 가감속 튜닝 변수 (로봇 무게에 맞춰 수정 가능!)
    // min_speed: 모터가 멈추지 않고 최소한으로 움직일 수 있는 힘 (너무 작으면 모터가 웅~ 소리만 내고 못 돎)
    double min_speed = 0.15; 
    
    // ramp_zone: 전체 거리 중 앞뒤로 몇 %를 가감속에 쓸 것인지 설정 (현재 25%)
    double ramp_zone = target_tick * 0.25; 
    
    // 만약 이동 거리가 너무 짧으면 그냥 절반씩 가감속 구간으로 나눔
    if (ramp_zone < 10) ramp_zone = target_tick / 2.0; 

    while (!::ems && !::Disable) {
        double current_enc = Motor::encoder[3].GetDistance();
        
        // 지금까지 이동한 거리 (절댓값)
        double traveled = std::fabs(current_enc - start_enc); 

        // 1. 목표 거리에 도달하면 루프 탈출!
        if (traveled >= target_tick) break; 

        double current_power = max_speed;

        // 2. 부드러운 출발 (가속 구간: 0 ~ 25%)
        if (traveled < ramp_zone) {
            // 거리가 늘어날수록 min_speed에서 max_speed로 점점 파워가 세짐
            current_power = min_speed + (max_speed - min_speed) * (traveled / ramp_zone);
        }
        // 3. 부드러운 정지 (감속 구간: 75% ~ 100%)
        else if (target_tick - traveled < ramp_zone) {
            // 남은 거리가 줄어들수록 max_speed에서 min_speed로 점점 브레이크!
            current_power = min_speed + (max_speed - min_speed) * ((target_tick - traveled) / ramp_zone);
        }

        // 4. 방향을 곱해서 3번 모터에 최종 힘 전달
        ::motor[3] = current_power * dir;
        delay(20);
    }
    
    // 5. 목표 거리만큼 오면 확실하게 정지!
    ::motor[3] = 0;
}

// OMS.cpp 맨 아래 추가
void OMS::homing_rail() {
    // 1. 비상 정지(EMS) 상태이거나 로봇이 꺼져있으면 아예 실행하지 않음
    if (::ems || ::Disable) return;

    // 2. 뒤에서 Motor::Loop가 3번 모터를 건드리지 못하게 PID 제어 강제 종료
    task_value[OMS_Z] = -1;
    prev_z = -1;

    // 3. 리밋 스위치가 눌릴 때까지 '뒤로' 계속 이동
    // (만약 뒤로 가는 파워가 양수여야 한다면 -0.25를 0.25로 바꿔줘!)
    while (!::ems && !::Disable && !::oms_z_limit) {
        ::motor[3] = 0.25; 
        delay(20); // 루프가 너무 빨리 돌아서 뻗지 않도록 필수 휴식
    }

    // 4. 스위치가 눌려서 위 루프를 탈출했다면? -> '앞으로' 조금 이동
    if (!::ems && !::Disable) {
        ::motor[3] = -0.1;  // 앞으로 밀어주는 파워
        delay(100);         // 0.2초 동안 이동 (얼마나 튀어나올지에 따라 이 숫자를 100~300으로 조절!)

        // 5. 완벽하게 모터 끄기
        ::motor[3] = 0;     

        // 🌟 6. 가장 중요한 마무리: 튀어나온 지금 위치를 새로운 영점(0)으로 리셋!
        Motor::encoder[3].Reset();
    }
}