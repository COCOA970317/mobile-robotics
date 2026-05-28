#include "All.h"
#include <frc/smartdashboard/SmartDashboard.h>
//=================================================================================
// private

clock_t Move::start_time;
int Move::filter_value, Move::acc_value, Move::accw_value, Move::gyro_ch;
int Move::t_ga, Move::t_gw, Move::f_max;
double Move::speed, Move::wspeed, Move::acc, Move::accw, Move::Ga, Move::Gw, Move::stop_dis, Move::wstop_dis;
double Move::ve_l[3], Move::ve_d[3], Move::ve_x[3], Move::ve_y[3];
double Move::rx, Move::ry, Move::rw;
double Move::local_en[3], Move::global_en[3][3];

double Move::filter_gain=-1;

void Move::Accelation(){
    f_max++;
    if(f_max>0) f_max=0;

    t_ga++;
    t_gw++;

    // 기존 선형 가속 대신 S-Curve 적용
    // S-Curve Formula: 3t^2 - 2t^3 (0 ≤ t ≤ 1)
    double t_normalized_ga = std::min(1.0, (double)t_ga / acc);
    double t_normalized_gw = std::min(1.0, (double)t_gw / accw);
    
    // S-Curve 적용 (jerk 최소화)
    Ga = 3.0 * t_normalized_ga * t_normalized_ga - 
         2.0 * t_normalized_ga * t_normalized_ga * t_normalized_ga;
    Gw = 3.0 * t_normalized_gw * t_normalized_gw - 
         2.0 * t_normalized_gw * t_normalized_gw * t_normalized_gw;
    
    // 더 부드러운 5차 다항식 버전 (옵션)
    // Ga = 10*t^3 - 15*t^4 + 6*t^5
    // Ga = 10.0 * pow(t_normalized_ga, 3) - 
    //      15.0 * pow(t_normalized_ga, 4) + 
    //       6.0 * pow(t_normalized_ga, 5);
}
void Move::reset(int sp, int wsp, int gg, int filter){
    acc = Trans(70, 534, 80, 120, sp);
    stop_dis = Trans(70, 534, 0, 300, sp);

    if(acc_value) acc=acc_value;

    accw = Trans(20, 140, 80, 120, wsp); // 회전도 부드럽게
    wstop_dis = Trans(20, 140, 0, 90, wsp);

    if(filter==1) filter=0;
    else if(filter==0 && gg/10==0) filter=-200;

    f_max=filter;

    t_ga=t_gw=0;
    Ga=Gw=0;
    
    if(gg/10==0){
        Ga=1;
        t_ga=5000;
    }
    if(gg/100){
        Gw=1;
        t_gw=5000;
    }

    for(int i=0;i<3;i++) local_en[i]=0;
    px[0]=py[0]=pw[0]=0;
    ::yaw[0]=0;
    start_time = clock();
}
void Move::VT(double f_agl, double len, int ch){
    ve_x[ch]=len*cos(f_agl*(M_PI/180)); // 0.017453
    ve_y[ch]=len*sin(f_agl*(M_PI/180));
}
void Move::len_VT(double x, double y, int ch){
    ve_l[ch]=hypot(x,y);
    ve_d[ch]=atan2(y,x)*180/M_PI;
}
int Move::get_mode(int f_agl, int &num){
    int mode=0;

    if(f_agl<0) f_agl+=360;
    
    if(num==LF || num==LB){
        if(f_agl>=180) mode=1;
        else mode=2;
    }
    else if(num==RF){ // RF (Right Front) 센서 로직
        if(f_agl>=180) mode=2;
        else mode=1;
    }
    else if(num==BS){ // BS (Back Sensor) 센서 로직 수정
        if(f_agl>=180) mode=1; // 후진(f_agl=180)할 때는 값이 '작아질 때' 멈춤 (mode=1)
        else mode=2; // 전진할 때는 값이 '커질 때' 멈춤 (mode=2)
    }
    else if(num==FL || num==FR){
        if(f_agl>=270 || f_agl<=90) mode=1;
        else mode=2;
    }
    else if(num==COB){
        num=0x06;
        mode=3;
    }
    else if(num==COB_LOST){
        num=0x06;
        mode=4;
    }
    else if(num==FLR){
        mode=6;
    }
    
    else if(num == XLF) {
        num = LF;
        mode = 7;
    }
    else if(num == XFL) {
        num = FL;
        mode = 7;
    }
    else if(num == XFR) {
        num = FR;
        mode = 7;
    }
    else if(num == XRF) {
        num = RF;
        mode = 7;
    }
    else if(num == XBS) {
        num = BS;         
        mode = 7;         
    }
    return mode;
}
int Move::Mode(double len, int sp, int gg, int mode, int num, int value, int mm){
    int S=::cobra;

    if(gg%10){
        if(mode==1 || mode==2){
            if(fabs(sensor[num]-value)<150) speed=Trans(0,150,70,sp,fabs(sensor[num]-value));
            else speed=sp;
        }
        else if(mode==6){
            double dis1, dis2, dis;

            dis1=fabs(sensor[FL]-value);
            dis2=fabs(sensor[FR]-value);

            if(dis1<dis2) dis=dis1;
            else dis=dis2;

            speed=Trans(0,stop_dis+90,70,sp,dis);
        }
        else if(mode==7) speed=Trans(len-stop_dis,len,sp,400,ve_l[1]);
        else if(mode==8) speed=Trans(len-stop_dis,len,sp,150,ve_l[1]);
        else speed=Trans(len-stop_dis,len,sp,70,ve_l[1]);
    }
    else speed=sp;
    
    if(ve_l[1]>=mm){
        if(mode==0 && ve_l[1]>=len) return 1;
        else if(mode==1 && sensor[num]<=value) return 1;
        else if(mode==2 && sensor[num]>=value) return 1;
        else if(mode==3 && (S&num)==num) return 1;
        else if(mode==4 && (S&num)==0) return 1;
        else if(mode==5 && (S&num)) return 1;
        else if(mode==6 && (sensor[FL]<=value || sensor[FR]<=value)) return 1;
        else if(mode==7 && sensor[num]>=value) return 1;
    }
    if((clock()-start_time)/CLOCKS_PER_SEC >= Constants::TIMEOUT_SEC) return 1;
    return 0;
}
void Move::end(){
    filter_value=0;
    acc_value=0;
    filter_gain=-1;

    Motor::pid[0].Reset();
    Motor::pid[1].Reset();
    Motor::pid[2].Reset();
}

//=================================================================================
// public

void Move::hold(){
    stop();
    while(!::ems && ::sw1) delay(20);
    while(!::ems && !::sw1) delay(20);
    while(!::ems && ::sw1) delay(20);
    start_time=clock();
}

void Move::stop(){
    ::motor[0]=::motor[1]=::motor[2]=0;

    Motor::motor[0].Set(0);
    Motor::motor[1].Set(0);
    Motor::motor[2].Set(0);

    Motor::pid[0].Reset();
    Motor::pid[1].Reset();
    Motor::pid[2].Reset();

    ::Motor_Reset = true;
}

void Move::NH(double x, double y, double w){
    double sp[3]={0,}, max=0;
    
    sp[0] = x*(sqrt(3)/2) + y/2 + w*3.4;
    sp[1] = -x*(sqrt(3)/2) + y/2 + w*3.4;
    sp[2] = -y + w*3.4;

    max = fabs(sp[0]);
    if(fabs(sp[1]) > max) max = fabs(sp[1]);
    if(fabs(sp[2]) > max) max = fabs(sp[2]);

    if(max > 534){
        sp[0] /= max/534;
        sp[1] /= max/534;
        sp[2] /= max/534;
    }

    if(::sw1) hold();

    ::motor[0] = Trans(-534,534,-1,1,sp[0]);
    ::motor[1] = Trans(-534,534,-1,1,sp[1]);
    ::motor[2] = Trans(-534,534,-1,1,sp[2]);
}
void Move::HD(double x, double y, double w, int filter){
    double k=8;
    
    if(filter_gain==-1){
        k=8;
        if(x<rx-k) k=5; // +x, y로 이동할 때 완만하게
    }
    else k=filter_gain;
    
    if(filter){
        if(x>rx+k) rx+=k;
        else if(x<rx-k) rx-=k;
        else rx=x;

        if(y>ry+k) ry+=k;
        else if(y<ry-k) ry-=k;
        else ry=y;
    }
    else{
        rx=x;
        ry=y;
        rw=w;
    }
    if(x==rx && y==ry) f_max=0; // 목표값에 도달하면 필터 off
    NH(rx,ry,w);
}
double Move::get_agl(int x, int y){
    len_VT(x,y,0);
    return ve_d[0];
}
double Move::get_len(int x, int y){
    len_VT(x,y,0);
    return ve_l[0];
}

Move Move::set_acc(int ms){
    acc_value=ms/10;
    return Move::instance;
}
void Move::dec(int ms){
    clock_t past;
    double prev_sp[3]={rx, ry, rw}, sp[3]={0,};
    
    reset(100,0,11,0);

    for(int i=0;i<ms/10;i++){
        if(::ems) break;
        past = clock();

        if(prev_sp[0]) sp[0] = Trans(0,ms/10-1,prev_sp[0],70,i);
        if(prev_sp[1]) sp[1] = Trans(0,ms/10-1,prev_sp[1],70,i);
        if(gyro_ch>=0) sp[2] = gyro[gyro_ch];
        else sp[2]=0;

        HD(sp[0], sp[1], sp[2], 0);

        loop_delay(10, past);
    }
}
Move Move::set_filter(int filter){
    filter_value=filter;
    return Move::instance;
}
Move Move::filter_off(){
    filter_value=-1;
    return Move::instance;
}
Move Move::set_gyro(int ch, int heading){
    gyro_ch=ch;
    ::gyro_heading=heading;
    return Move::instance;
}
Move Move::gyro_off(){
    return set_gyro(-1);
}

// Basic Move
void Move::gyro_reset(){
    ::p_yaw[0]=::n_yaw;
}
void Move::position(int set){
    static const double distancePerTick = (M_PI * 2 * 51) / 1464;
    static double past_en[3];
    double x, y, w, cycle_en[3];

    if(set){
        px[set]=py[set]=pw[set]=0;
        global_en[0][set-1]=global_en[1][set-1]=global_en[2][set-1]=0;
        ::yaw[set]=0;
        ::p_yaw[set]=::n_yaw;
        ::input_w[set]=0;
    }
    else{
        for(int i=0;i<3;i++){
            local_en[i]+=::encoder[i]-past_en[i];
            cycle_en[i]=::encoder[i]-past_en[i];
            for(int j=0;j<3;j++) global_en[i][j]+=::encoder[i]-past_en[i];
            past_en[i]=::encoder[i];
        }

        px[0]=(local_en[0]-local_en[1])/2.0/(sqrt(3)/2)*distancePerTick;
        py[0]=(local_en[0]+local_en[1]-local_en[2]*2.0)/3.0*distancePerTick;

        x=(cycle_en[0]-cycle_en[1])/2.0/(sqrt(3)/2)*distancePerTick;
        y=(cycle_en[0]+cycle_en[1]-cycle_en[2]*2.0)/3.0*distancePerTick;

        for(int i=0;i<3;i++){
            w=::yaw[i+1];

            VT(w,x,0);
            VT(w,y,1);

            px[i+1]+=ve_x[0]-ve_y[1];
            py[i+1]+=ve_x[1]+ve_y[0];

            pw[i+1]=w;
        }
    }
}
void Move::global(int ch, int sp, int wsp, int gg, int x, int y, int w, int turn_dir){
    clock_t past;
    double ox, oy, ow;
    int b1=0, b2=0;

    if(!wsp) b2=1;
    reset(sp,wsp,gg,-filter_value);

    if(ch==4) position(3), ch=3;

    ow=w-::yaw[ch];
    if(fabs(ow)>180) ow = (360 - fabs(ow)) * -sign(ow);
    if(turn_dir && sign(ow) != turn_dir) ow = -ow;

    for(int i=0;i<4;i++){
        if(i!=ch){
            ::input_w[i] += ow;
        }       
    }
    ::input_w[ch] = w;
    
    while(!::ems){
        past=clock();

        ox=x-px[ch];
        oy=y-py[ch];
        ow=w-yaw[ch];
        if(fabs(ow)>180) ow = (360 - fabs(ow)) * -sign(ow);
        if(turn_dir && sign(ow) != turn_dir) ow = -ow;

        len_VT(ox,oy,0);

        Accelation();
        if(gg%10) speed=Trans(0,stop_dis+10,70,sp,ve_l[0]);
        else speed=sp;

        if(wsp) wspeed=Trans(0,wstop_dis+10,20,fabs(wsp),fabs(ow))*(ow>0 ? 1:-1);
        else {
            // [수정 포인트] 0으로 나누기 방지!
            // 거리가 1mm도 안 남았으면 나눗셈 하지 말고 회전 속도 0
            if(ve_l[0] < 1.0) wspeed = 0;
            else wspeed=(speed*ow/ve_l[0]);
        }

        if(ve_l[0]<10 || b1) b1=1, speed=0;
        if(fabs(ow)<1) b2=1, wspeed=0;
        if(b1 && b2) break;
        if((clock()-start_time)/CLOCKS_PER_SEC >= Constants::TIMEOUT_SEC) break;

        VT(ve_d[0]-yaw[ch],speed*Ga,0);
        HD(ve_x[0],ve_y[0],wspeed*Gw, f_max);

        loop_delay(10, past);
    }
}
void Move::move(int agl, int sp, int gg, int x, int y){
    clock_t past;
    double errW;

    len_VT(y,x,0);
    reset(sp,0,gg,-filter_value);

    while(!::ems){
        past=clock();

        len_VT(px[0],py[0],1);

        if(Mode(ve_l[0],sp,gg,0,0,0,0)) break; // 감속
        
        Accelation(); // 가속

        if(gyro_ch>=0) errW=gyro[gyro_ch];
        else errW=0;

        VT(ve_d[0]+agl,speed*Ga,0);
        HD(ve_x[0],ve_y[0],errW,f_max);

        loop_delay(10, past);
    }
    end();
}
void Move::turn(int sp, int gg, int w){
    clock_t past;

    for(int i=0;i<4;i++){
        ::input_w[i]+=w;
        if(::input_w[i]>359) ::input_w[i] -= 360;
        else if(::input_w[i]<0) ::input_w[i] += 360;
    }

    ve_l[0]=fabs(w);
    reset(0,sp,gg,-filter_value);

    while(!::ems){
        past=clock();

        ve_l[1]=fabs(yaw[0]);
        if(Mode(ve_l[0],sp,gg,0,0,0,0)) break; // 감속
        
        Accelation(); // 가속

        if(gg%10) wspeed=Trans(ve_l[0]-wstop_dis,ve_l[0],sp,10,ve_l[1]);
        else wspeed=sp;
        
        HD(0,0,wspeed*Gw*(w>0?1:-1),f_max); // w
        loop_delay(10, past);
    }
    end();
}
void Move::holonomic(int agl, int sp, int gg, int x, int y, int w){
    clock_t past;
    double ow;

    len_VT(y,x,0);
    position(3);

    for(int i=0;i<4;i++){
        ::input_w[i]+=w;
        if(::input_w[i]>359) ::input_w[i] -= 360;
        else if(::input_w[i]<0) ::input_w[i] += 360;
    }

    reset(sp,sp,gg,-filter_value);

    while(!::ems){
        past=clock();

        len_VT(px[3],py[3],1);
        
        // ═══════════════════════════════════════════════════════════════
        // ▼▼▼ [핵심 수정] 각도 오차 계산을 먼저 수행 ▼▼▼
        // ═══════════════════════════════════════════════════════════════
        ow = w - ::yaw[3];
        if(ow > 180) ow -= 360;
        else if(ow < -180) ow += 360;
        
        // ═══════════════════════════════════════════════════════════════
        // ▼▼▼ [핵심 수정] 종료 조건: 거리 AND 각도 모두 체크 ▼▼▼
        // ═══════════════════════════════════════════════════════════════
        bool distance_ok = Mode(ve_l[0], sp, gg, 0, 0, 0, 0);
        bool rotation_ok = (fabs(ow) < 2.0); // 각도 오차 2도 이내
        
        // 둘 다 충족 시 종료
        if(distance_ok && rotation_ok) break;
        
        // 거리는 도달했는데 각도가 안 맞으면 → 제자리 회전만 수행
        // 각도만 도달했는데 거리가 안 맞으면 → 직진만 수행
        // ═══════════════════════════════════════════════════════════════
        
        Accelation();

        // ═══════════════════════════════════════════════════════════════
        // ▼▼▼ 이동 속도 계산 (기존 로직 유지) ▼▼▼
        // ═══════════════════════════════════════════════════════════════
        double remaining_dist = ve_l[0] - ve_l[1];

        // 거리 도달 시 이동 중단
        if(distance_ok || remaining_dist < 2){
            speed = 0; // 이동 속도 0
        }
        
        // ═══════════════════════════════════════════════════════════════
        // ▼▼▼ 회전 속도 계산 (수정된 로직) ▼▼▼
        // ═══════════════════════════════════════════════════════════════
        if(rotation_ok){
            wspeed = 0; // 각도 도달 → 회전 중단
        }
        else if(remaining_dist < 1.0){
            // 거리가 거의 없으면 회전만 수행 (속도 독립)
            // gg%10이 1이면 감속 모드 → 부드러운 회전
            if(gg % 10){
                wspeed = Trans(0, 30, 20, sp*0.3, fabs(ow)) * (ow > 0 ? 1 : -1);
            }
            else{
                wspeed = sp * 0.3 * (ow > 0 ? 1 : -1); // 일정 속도
            }
        }
        else{
            // 정상 회전: 거리에 비례하여 분배
            wspeed = (speed * (ow) / remaining_dist);
        }
        
        // ═══════════════════════════════════════════════════════════════
        // ▼▼▼ [안전 장치] 회전 방향 검증 ▼▼▼
        // ═══════════════════════════════════════════════════════════════
        // 목표 각도와 실제 회전 방향이 반대면 → 즉시 중단
        // (오버슈트 후 역회전 방지)
        if((ow > 0 && wspeed < 0) || (ow < 0 && wspeed > 0)){
            wspeed = 0;
        }
        
        // 최대 회전 속도 제한 (폭주 방지)
        double max_wspeed = sp * 0.8;
        if(wspeed > max_wspeed) wspeed = max_wspeed;
        else if(wspeed < -max_wspeed) wspeed = -max_wspeed;
        // ═══════════════════════════════════════════════════════════════

        VT(ve_d[0]-yaw[3]+agl,speed*Ga,0);
        HD(ve_x[0],ve_y[0],wspeed*Ga,f_max);
        
        loop_delay(10, past);

        frc::SmartDashboard::PutNumber("Holo ow", ow);
        frc::SmartDashboard::PutNumber("Holo wspeed", wspeed);
        frc::SmartDashboard::PutNumber("Holo dist", remaining_dist);
    }

    for(int i=0;i<4;i++){
        ::p_yaw[i] = ::n_yaw;
        ::yaw[i] = 0;
        ::input_w[i] = 0;
    }

    end();
}

void Move::cp(int value, int sub_num, int sub_value, double ratio){
    clock_t past, end_time;
    bool timer_flg=false, w_flg=true;
    int danger_cnt=0;
    double errX=0, errY=0, errW=0, x=0, y=0, w=0;

    reset(300,0,11,-filter_value);

    while(!::ems){
        past=clock();
        
        if(value){
            if(fabs(::sensor[FL]-value) < fabs(::sensor[FR]-value)) errX=(::sensor[FL]-value);
            else errX=(::sensor[FR]-value);
            errW=(::sensor[FL]-::sensor[FR]) + ratio;
        }
        else if(gyro_ch>=0){
            if(value) errX=(::sensor[FL]-value);
            w=gyro[gyro_ch];
            w_flg=false;
        }

        if(sub_value){
            if(sub_num==LF) errY=-(::sensor[LF]-sub_value);
            else if(sub_num==RF) errY=(::sensor[RF]-sub_value);
        }

        if(fabs(errX)<10 && fabs(errY)<10 && fabs(errW)<6){ //cp 정도도
            if(!timer_flg){
                end_time=clock();
                timer_flg=true;
            }
            else if(timer_flg && (clock()-end_time)/1000 >= 300) break;
        }
        else if(timer_flg) timer_flg=false;

        if((clock()-start_time)/CLOCKS_PER_SEC >= Constants::TIMEOUT_SEC) break;

        x=Trans(0,350,50,350,fabs(errX)) * (errX>0 ? 1 : -1);
        y=Trans(0,350,40,400,fabs(errY)) * (errY>0 ? 1 : -1);
        if(w_flg) w=Trans(10,50,5,25,fabs(errW)) * (errW>0 ? 1 : -1);

        if(sub_value==0 || !w_flg){
            x*=1.1;
            y*=1.1;
        }

        if(value && fabs(errX)>100) w=0;
        if(fabs(errW)>100) danger_cnt=100;
        if(danger_cnt){
            w=0;
            danger_cnt--;
        }
        
        if(fabs(errX)<4) x=0;
        if(fabs(errY)<4) y=0;
        if(fabs(errW)<3) w=0;

        Accelation();
        HD(x*Ga,y*Ga,w,f_max);

        loop_delay(10,past);
    }
    stop();
    end();
}
void Move::align_parallel(int value, double ratio){
    clock_t past, end_time;
    bool timer_flg=false;
    double errW=0, w=0;

    // 회전 속도만 50으로 설정 (이동 속도는 0)
    reset(0, 50, 11, -filter_value);

    while(!::ems){
        past=clock();
        
        // cp() 함수에서 각도 오차(errW) 계산 로직만 가져옴
        if(value == FLR){
             errW=(::sensor[FL]-::sensor[FR]) + ratio;
        }
        else{
            // 다른 센서 조합(예: LF, RF)이 필요하면 여기에 추가
            break; // 현재는 FLR만 지원
        }

        // cp()의 정지 조건과 동일
        if(fabs(errW)<6){
            if(!timer_flg){
                end_time=clock();
                timer_flg=true;
            }
            else if(timer_flg && (clock()-end_time)/1000 >= 300) break;
        }
        else if(timer_flg) timer_flg=false;

        if((clock()-start_time)/CLOCKS_PER_SEC >= Constants::TIMEOUT_SEC) break;

        // cp()의 회전(w) 계산 로직과 동일
        w=Trans(10,50,5,25,fabs(errW)) * (errW>0 ? 1 : -1);
        if(fabs(errW)<3) w=0;

        Accelation();
        // X와 Y는 0으로 고정하고, W(회전)만 명령함
        HD(0, 0, w, f_max);

        loop_delay(10,past);
    }
    stop();
    end();
}
void Move::agl_sensor(int f_agl, int sp, int gg, int num, int value, int mm, int cnt){
    clock_t past;
    double errW=0;
    int mode=1;
    int loop_count = 0; // <--- [추가] 루프 카운터 변수

    reset(sp,0,gg,-filter_value);

    mode=get_mode(f_agl,num);

    while(!::ems){
        past=clock();
        loop_count++; // <--- [추가] 루프 1회 실행마다 카운트 증가

        len_VT(px[0],py[0],1); // 현재 이동 거리(ve_l[1]) 계산

        bool stop_condition_met = false;
        
        // 1. 'cnt' 횟수만큼 무시했는지 확인
        if (loop_count > cnt) { 
            
            // 2. 'cnt'를 통과했다면, Mode() 함수를 호출
            // Mode() 함수는 내부적으로 'mm' 거리만큼 무시했는지
            // (ve_l[1] >= mm)를 스스로 검사합니다.
            if(Mode(value, sp, gg, mode, num, value, mm)) {
                stop_condition_met = true;
            }
        }
        
        if (stop_condition_met) break; // 정지 조건이 만족되면 루프 탈출
        
        Accelation(); // 가속

        if(gyro_ch>=0) errW=gyro[gyro_ch];

        VT(f_agl,speed*Ga,0);
        HD(ve_x[0],ve_y[0],errW,f_max);
        
        loop_delay(10, past);
    }
    end();
}

void Move::left_wall_move(int ta, int sp, int gg, int num, int value, int mm){
    clock_t past;
    double errX=0, errY=0, errW=0;
    int f_agl, mode=1, flg=0;

    f_agl = sp<0 ? 180: 0;
    sp = abs(sp);

    mode = get_mode(f_agl,num);
    reset(sp,0,gg,-filter_value);
    while(!::ems){
        past = clock();
        
        len_VT(px[0],0,1);
        if(Mode(value,sp,gg,mode,num,value,mm)) break; // 감속
        Accelation(); // 가속

        errY = (ta-::sensor[LF]);

        if(fabs(errY)<60) flg=1;
        if(flg) errY*=1.5;
        else errY*=2;
        
        //if(f_agl==180) errY*=2.3;

        if(gyro_ch==-1) errW=0;
        else errW=gyro[gyro_ch];
     
        VT(f_agl,speed*Ga,0);
        HD(ve_x[0] + errX*Ga, ve_y[0] + errY*Ga, errW*Ga, f_max);

        loop_delay(10, past);
    }
    end();
}

void Move::right_wall_move(int ta, int sp, int gg, int num, int value, int mm){
    clock_t past;
    double errX=0, errY=0, errW=0;
    int f_agl, mode=1, flg=0;

    f_agl = sp<0 ? 180:0;
    sp = abs(sp);

    mode = get_mode(f_agl,num);
    reset(sp,0,gg,-filter_value);
    while(!::ems){
        past = clock();
        
        len_VT(px[0],0,1);
        if(Mode(value,sp,gg,mode,num,value,mm)) break; // 감속
        Accelation(); // 가속

        errY = (::sensor[RF]-ta);

        if(fabs(errY)<60) flg=1;
        if(flg) errY*=1.5;
        else errY*=2;

        //if(f_agl==180) errY*=2.3;

        if(gyro_ch==-1) errW=0;
        else errW=gyro[gyro_ch];
     
        VT(f_agl,speed*Ga,0);
        HD(ve_x[0] + errX*Ga, ve_y[0] + errY*Ga, errW*Ga, f_max);

        loop_delay(10, past);
    }
    end();
}
void Move::FL_wall_move(int ta, int sp, int gg, int num, int value, int mm){
    clock_t past;
    double errX=0, errY=0, errW=0; // errY는 사용되지 않음
    int f_agl, mode=1, flg=0;

    f_agl = sp<0 ? -90:90; // sp 음수 = 오른쪽(-90도), sp 양수 = 왼쪽(90도)
    sp = abs(sp);

    mode = get_mode(f_agl,num);
    reset(sp,0,gg,-filter_value);

    while(!::ems){
        past = clock();
        
        len_VT(0,py[0],1); // Y축(좌/우)으로 이동한 거리 계산
        if(Mode(value,sp,gg,mode,num,value,mm)) break; // 종료 조건 확인
        Accelation(); // 가속

        // --- [수정된 로직 1] ---
        // X축(앞/뒤) 보정은 오직 FL 센서 값만 사용
        errX = (ta - ::sensor[FL]);

        if(fabs(errX)<60) flg=1;
        if(flg) errX*=1.2;
        else errX*=2;

        // --- [수정된 로직 2] ---
        // W축(회전) 보정은 자이로 값만 사용
        if(gyro_ch==-1) errW=0;
        else errW=gyro[gyro_ch]; //
       
        VT(f_agl,speed*Ga,0); // 좌/우 기본 이동 속도 계산
        
        // HD 함수가 Y축(speed), X축(errX), 회전(errW)을 모두 합쳐 모터로 보냄
        HD(ve_x[0] + errX*Ga, ve_y[0] + errY*Ga, errW*Ga, f_max);

        loop_delay(10, past);
    }
    end();
}
void Move::FR_wall_move(int ta, int sp, int gg, int num, int value, int mm){
    clock_t past;
    double errX=0, errY=0, errW=0; // errY는 사용되지 않음
    int f_agl, mode=1, flg=0;

    f_agl = sp<0 ? -90:90; // sp 음수 = 오른쪽(-90도), sp 양수 = 왼쪽(90도)
    sp = abs(sp);

    mode = get_mode(f_agl,num);
    reset(sp,0,gg,-filter_value);

    while(!::ems){
        past = clock();
        
        len_VT(0,py[0],1); // Y축(좌/우)으로 이동한 거리 계산
        if(Mode(value,sp,gg,mode,num,value,mm)) break; // 종료 조건 확인
        Accelation(); // 가속

        // --- [수정된 로직 1] ---
        // X축(앞/뒤) 보정은 오직 FR 센서 값만 사용
        errX = (ta - ::sensor[FR]);

        if(fabs(errX)<60) flg=1;
        if(flg) errX*=1.2;
        else errX*=2;

        // --- [수정된 로직 2] ---
        // W축(회전) 보정은 자이로 값만 사용
        if(gyro_ch==-1) errW=0;
        else errW=gyro[gyro_ch]; //
       
        VT(f_agl,speed*Ga,0); // 좌/우 기본 이동 속도 계산
        
        // HD 함수가 Y축(speed), X축(errX), 회전(errW)을 모두 합쳐 모터로 보냄
        HD(ve_x[0] + errX*Ga, ve_y[0] + errY*Ga, errW*Ga, f_max);

        loop_delay(10, past);
    }
    end();
}
void Move::front_wall_move(int ta, int sp, int gg, int num, int value, int mm){
    clock_t past;
    double errX=0, errY=0, errW=0;
    int f_agl, mode=1, flg=0;

    f_agl = sp<0 ? -90:90;
    sp = abs(sp);

    mode = get_mode(f_agl,num);
    reset(sp,0,gg,-filter_value);
    while(!::ems){
        past = clock();
        
        len_VT(0,py[0],1);
        if(Mode(value,sp,gg,mode,num,value,mm)) break; // 감속
        Accelation(); // 가속

        if(f_agl==90) errX = (::sensor[FR]-ta);
        else errX = (::sensor[FL]-ta);

        if(fabs(errX)<60) flg=1;
        if(flg) errX*=1.2;
        else errX*=2;

        // if(gyro_ch==-1) errW=0;
        // else errW=gyro[gyro_ch];
        errW = (::sensor[FL]-::sensor[FR])*1.5;
       
        VT(f_agl,speed*Ga,0);
        HD(ve_x[0]+errX*Ga,ve_y[0]+errY*Ga,errW*Ga,f_max);

        loop_delay(10, past);
    }
    end();
}
void Move::back_wall_move(int ta, int sp, int gg, int num, int value, int mm){
    clock_t past;
    double errX=0, errY=0, errW=0; // errY는 사용되지 않습니다.
    int f_agl, mode=1, flg=0;

    f_agl = sp<0 ? -90:90; // sp 음수 = 오른쪽(-90도), sp 양수 = 왼쪽(90도)
    sp = abs(sp);

    mode = get_mode(f_agl,num);
    reset(sp,0,gg,-filter_value);

    while(!::ems){
        past = clock();
        
        len_VT(0,py[0],1); // Y축(좌/우)으로 이동한 거리 계산
        if(Mode(value,sp,gg,mode,num,value,mm)) break; // 종료 조건 확인
        Accelation(); // 가속

        // [수정 전 코드]
        // 목표값(ta)에서 센서값(BS)을 뺍니다.
        errX = (ta - ::sensor[BS]);

        // 2. 오차에 따른 게인(Gain) 적용
        if(fabs(errX)<60) flg=1;
        if(flg) errX*=1.2;
        else errX*=2;

        // 3. 각도 보정 (자이로 사용)
        if(gyro_ch==-1) errW=0;
        else errW=gyro[gyro_ch];
       
        VT(f_agl,speed*Ga,0); // 좌/우 기본 이동 속도 계산
        
        // HD 함수가 X축(errX), Y축(speed), 회전(errW)을 모두 합쳐 모터로 보냅니다.
        HD(ve_x[0] + errX*Ga, ve_y[0] + errY*Ga, errW*Ga, f_max);

        loop_delay(10, past);
    }
    
    Move::stop();
    end();
}
void Move::line_find(int f_agl, int sp, int gg, int n, int value, int mm){
    clock_t past;
    double errW=0;
    int flg=0, cnt=0, num=6, mode=3;
    bool auto_dec=false;

    if(n) mode=999;

    if(value==0 && gg%10){
        gg=gg/10*10;
        auto_dec=true;
    }

    if(f_agl<0 || f_agl>180){
        if(auto_dec) num=2;
        else num=4;
    }
    else if(f_agl>0 || f_agl<180){
        if(auto_dec) num=4;
        else num=2;
    }

    reset(sp,0,gg,-filter_value);

    while(!::ems){
        past=clock();

        len_VT(px[0],py[0],1);

        if(ve_l[1]>=mm && n){
            if(!flg && (::cobra & num)){
                cnt++;
                flg=1;
            }
            else if(flg && !(::cobra & num)) flg=0;

            if(cnt>=n) break;
        }

        if(Mode(value,sp,gg,mode,num,0,mm)) break; // 감속
        Accelation(); // 가속

        if(gyro_ch>=0) errW=gyro[gyro_ch];

        VT(f_agl,speed*Ga,0);
        HD(ve_x[0],ve_y[0],errW,f_max);
        
        loop_delay(10, past);
    }
    if(auto_dec){ // 자동 감속
        int sp=speed, ms=70;
        for(int i=0;!::ems && i<ms/10;i++){
            past=clock();

            speed=Trans(0,ms/10-1,sp,ms,i);

            if(gyro_ch>=0) errW=gyro[gyro_ch];

            VT(f_agl,speed*Ga,0);
            HD(ve_x[0],ve_y[0],errW,f_max);

            loop_delay(10, past);
        }
    }
    end();
}
void Move::line_lost(int f_agl, int sp, int gg, int value, int mm){
    clock_t past;
    double errW=0;
    int mode=4;

    if(value==0) gg=gg/10*10;

    reset(sp,0,gg,-filter_value);

    while(!::ems){
        past=clock();
        
        len_VT(px[0],py[0],1);
        if(Mode(value,sp,gg,mode,0x06,0,mm)) break; // 감속
        Accelation(); // 가속

        if(gyro_ch>=0) errW=gyro[gyro_ch];

        VT(f_agl,speed*Ga,0);
        HD(ve_x[0],ve_y[0],errW,f_max);
        
        loop_delay(10, past);
    }
    end();
}

void Move::line(int sp, int gg, int num, int value, int mm){
    clock_t past; 
    double errY=0, errW=0;
    int f_agl=0, S=0, mode=0;

    if(sp<0) f_agl=180, sp=-sp;

    mode=get_mode(f_agl,num);

    reset(sp,0,gg,-filter_value);

    while(!::ems){
        past = clock();
        
        len_VT(px[0],0,1);
        if(Mode(value,sp,gg,mode,num,value,mm)) break; // 감속
        Accelation(); // 가속

        S=::cobra;
        
        if((S&0x06)==0) errY=0;
        else if((S&0x02)==0) errY=20;
        else if((S&0x04)==0) errY=-20;
        else errY=0;

        if(gyro_ch>=0) errW=gyro[gyro_ch];
        else errW=0;

        if(f_agl==0) HD(speed*Ga,errY,errW,f_max);
        else HD(-speed*Ga,errY*3,-errW*0.5,f_max);

        loop_delay(10, past);
    }
    end();
}

int Move::move_and_count_lines(int agl, int sp, int gg, int x, int y) {
    clock_t past;
    double errW;

    // --- 라인 카운팅을 위한 변수 ---
    int line_count = 0;      // 라인 개수 카운터
    bool is_on_line = false; // 현재 라인을 밟고 있는지 상태 플래그
    int line_sensor_num = 0x06; // 중앙 라인 센서 (필요시 수정)
    // --------------------------------

    len_VT(x, y, 0); // 목표 거리(ve_l[0]) 계산
    reset(sp, 0, gg, -filter_value);

    while (!::ems) {
        past = clock();

        len_VT(px[0], py[0], 1); // 현재 이동 거리(ve_l[1]) 계산

        // --- 'Move::move'의 정지 조건 ---
        if (Mode(ve_l[0], sp, gg, 0, 0, 0, 0)) {
            break; // 목표 거리에 도달하면 루프 탈출
        }
        // --------------------------------
        
        Accelation(); // 가속

        // --- 'Move::line_find'의 카운팅 로직 ---
        if (!is_on_line && (::cobra & line_sensor_num)) {
            // 라인 밖 -> 안으로 진입하는 순간
            line_count++;         // 카운트 증가
            is_on_line = true;    // '라인 안' 상태로 변경
        } else if (is_on_line && !(::cobra & line_sensor_num)) {
            // 라인 안 -> 밖으로 벗어나는 순간
            is_on_line = false;   // '라인 밖' 상태로 리셋
        }
        // --------------------------------

        if (gyro_ch >= 0) errW = gyro[gyro_ch];
        else errW = 0;

        VT(ve_d[0] + agl, speed * Ga, 0);
        HD(ve_x[0], ve_y[0], errW, f_max);

        loop_delay(10, past);
    }
    
    end();
    
    return line_count; // 함수가 종료될 때 총 라인 개수를 반환
}
double Move::move_length(int f_agl, int sp, int gg, int num, int value, int mm, int cnt) {
    clock_t past;
    double errW = 0;
    int mode = 1;
    int loop_count = 0; 

    // reset() 함수는 로봇의 로컬 좌표(px[0], py[0])를 0으로 초기화합니다.
    reset(sp, 0, gg, -filter_value);

    mode = get_mode(f_agl, num);

    while (!::ems) {
        past = clock();
        loop_count++; 

        // len_VT()는 현재 로컬 좌표(px[0], py[0])를 기반으로
        // reset() 이후 총 이동한 거리(ve_l[1])를 계산합니다.
        len_VT(px[0], py[0], 1); 

        bool stop_condition_met = false;
        
        // 'cnt' 횟수만큼 무시했는지 확인
        if (loop_count > cnt) { 
            // 'cnt'를 통과했다면, Mode() 함수를 호출
            // Mode() 함수는 내부적으로 'mm' 거리만큼 무시했는지
            // (ve_l[1] >= mm)를 스스로 검사합니다.
            if (Mode(value, sp, gg, mode, num, value, mm)) {
                stop_condition_met = true;
            }
        }
        
        if (stop_condition_met) break; // 정지 조건이 만족되면 루프 탈출
        
        Accelation(); // 가속

        if (gyro_ch >= 0) errW = gyro[gyro_ch];

        VT(f_agl, speed * Ga, 0);
        HD(ve_x[0], ve_y[0], errW, f_max);
        
        loop_delay(10, past);
    }
    
    end(); // 이동 관련 변수들 리셋

    // 루프가 종료된 시점의 최종 이동 거리(ve_l[1])를 반환합니다.
    return ve_l[1]; 
}
void Move::cam_tracking(int color_num, int target_x, int target_y){
    clock_t past;
    int cnt=0;
    double v[3];
    int errX, errY;

    static const double err_le[2][5]={ 
        {0, 4, 6,  40, 200},
        {0, 0, 35, 60, 250},
    };

    reset(100,0,11,-filter_value);

    while(!::ems){
        past = clock();

        errX = ShuffleBoard::nt_color_y.GetDouble(0);
        if(errX) errX = target_y - errX;
        else errX=0;
        errY = ShuffleBoard::nt_color_x.GetDouble(0);
        if(errY) errY = errY - target_x;
        else errY=0;

        Accelation();

        v[0] = Trans(Arr(err_le), errX);
        v[1] = Trans(Arr(err_le), errY);

        if(v[0]==0 && v[1]==0 && fabs(errX)<4 && fabs(errY)<4){
            cnt++;
            if(cnt>=5) break;
        }
        else cnt=0;

        if(gyro_ch>=0) v[2]=gyro[gyro_ch];
        else v[2]=0;

        HD(v[0]*Ga,v[1]*Ga,v[2]*Ga,f_max);
        delay(30 - (clock()-past)/1000);
    }
    Move::stop();
    end();
 
}
void Move::cp_front(int target_dist, int time_out){
    clock_t past, end_time;
    bool timer_flg = false;
    double errX = 0, errW = 0;
    double x = 0, w = 0;

    // 1. 초기화 (속도 300, 가감속 모드 11로 부드럽게)
    reset(300, 0, 11, -filter_value);

    while(!::ems){
        past = clock();

        // 2. 오차 계산
        // errX: (FL + FR) / 2 - 목표값 -> 앞뒤 거리 오차
        double current_avg = (::sensor[FL] + ::sensor[FR]) / 2.0;
        errX = current_avg - target_dist;

        // errW: FL - FR -> 틀어진 각도 오차 (0이 되어야 평행)
        errW = (::sensor[FL] - ::sensor[FR]);

        // 3. 정지 조건 (Success Condition)
        // 거리 오차가 4mm 이내이고, 각도 차이가 3mm 이내면 정지 타이머 시작
        if(fabs(errX) < 4 && fabs(errW) < 3){
            if(!timer_flg){
                end_time = clock();
                timer_flg = true;
            }
            // 0.3초 동안 안정적으로 유지되면 탈출
            else if((clock()-end_time)/1000 >= 300) break;
        }
        else timer_flg = false;

        // 타임아웃 체크
        if((clock()-start_time)/CLOCKS_PER_SEC >= time_out) break;

        // 4. 모터 출력 계산 (Trans 함수로 P제어)
        // 거리가 멀면 빨리, 가까우면 천천히 (최대속도 350, 최소속도 40)
        x = Trans(0, 350, 40, 350, fabs(errX)) * (errX > 0 ? 1 : -1);
        
        // 각도가 많이 틀어졌으면 빨리 회전 (최대회전 50, 최소회전 10)
        w = Trans(5, 50, 10, 50, fabs(errW)) * (errW > 0 ? 1 : -1);

        // 데드존 (미세 떨림 방지)
        if(fabs(errX) < 2) x = 0;
        if(fabs(errW) < 2) w = 0;

        Accelation(); // 가속 적용

        // 5. 실행 (Y축은 0)
        // HD(x, y, w, filter)
        HD(x*Ga, 0, w*Ga, f_max);

        loop_delay(10, past);
    }
    
    Move::stop();
    end();
}
void Move::front_wall_dist(int ta, int sp, int gg, int mm){
    clock_t past;
    double errX=0, errY=0, errW=0;
    double front_val = 0;
    int f_agl;

    // 1. 방향 설정 (속도가 +면 왼쪽, -면 오른쪽)
    f_agl = sp<0 ? -90:90;
    sp = abs(sp);

    // 2. 초기화 (거리 측정을 위해 reset 필수)
    reset(sp, 0, gg, -filter_value); 

    while(!::ems){
        past = clock();
        
        // 3. 현재 이동 거리 계산
        len_VT(px[3], py[3], 1); 

        // 4. 목표 거리(mm)에 도달했는지 확인 (Mode=0 : 거리 주행 모드)
        // gg(가감속)에 따라 부드럽게 멈춥니다.
        if(Mode(mm, sp, gg, 0, 0, 0, 0)) break; 

        Accelation(); // 가속 계산

        // 5. [요청사항] FL, FR 중 '더 높은 값'을 기준으로 거리 오차 계산
        if(::sensor[FL] > ::sensor[FR]) front_val = ::sensor[FL];
        else front_val = ::sensor[FR];

        errX = (front_val - ta); // 목표 거리(ta)와 비교

        // P제어: 거리가 틀어지면 보정 (너무 멀면 다가가고, 가까우면 물러남)
        // 반응성을 위해 게인값을 살짝 조정했습니다.
        if(fabs(errX) < 2) errX = 0; // 데드존 (2mm 오차는 무시)
        
        // 6. 각도 보정 (벽과 평행 유지)
        errW = (::sensor[FL] - ::sensor[FR]) * 1.5;

        // 7. 모터 출력
        // f_agl: 옆으로 가는 주행 힘
        // errX: 앞뒤 간격 맞추는 힘
        // errW: 비뚤어진 각도 잡는 힘
        VT(f_agl, speed*Ga, 0);
        HD(ve_x[0]+errX*Ga, ve_y[0]+errY*Ga, errW*Ga, f_max);

        loop_delay(10, past);
    }
    
    // 동작이 끝나면 깔끔하게 정지
    // (연속 동작을 원하시면 이 줄을 빼고 g_enter_speed 로직을 적용하면 됩니다)
    Move::stop(); 
    end();
}
void Move::agl_sensor_center(int f_agl, int sp, int gg, int num, int value, int cs, int mm, int cnt){
    clock_t past;
    double errW = 0;
    double center_force = 0; 
    double max_correction = 300; 

    int mode = 1;
    int loop_count = 0;

    reset(sp, 0, gg, -filter_value);

    mode = get_mode(f_agl, num);

    while(!::ems){
        past = clock();
        loop_count++;

        len_VT(px[0], py[0], 1); 

        bool stop_condition_met = false;
        if (loop_count > cnt) { 
            if(Mode(value, sp, gg, mode, num, value, mm)) stop_condition_met = true;
        }
        if (stop_condition_met) break;
        
        Accelation(); 

        // ---------------------------------------------------------
        // ▼ 벽 탈출 쇼크 방지 로직 적용
        // ---------------------------------------------------------
        
        // 1. 기본 조건: 둘 다 설정 거리(cs) 안에 있어야 함
        int num1 =::sensor[LF];
        int num2 = ::sensor[RF];
        if((num1 + num2) <= cs*2 && ::sensor[LF] > 10 && ::sensor[RF] > 10) {
            
            // 2. 오차 계산
            double diff = ::sensor[LF] - ::sensor[RF];

            // 3. [핵심 추가] "오차 급발진 방지" (Consistency Check)
            // 좌우 차이가 200mm(20cm) 이상 나면, 이건 기울어진 게 아니라 한쪽 벽이 없는 것임.
            // 이때는 보정을 하지 말고 직진만 해야 함.
            if(abs(diff) > 200) { 
                center_force = 0; // 벽 끝이니까 보정 포기!
            }
            else {
                // 정상 범위 오차일 때만 보정 수행
                // [부호 주의] 아까 수정한대로 마이너스(-) 붙여서 반대 방향으로 밀기
                center_force = -diff * 1.5; 

                // 힘 제한 (Clamping)
                if(center_force > max_correction) center_force = max_correction;
                else if(center_force < -max_correction) center_force = -max_correction;
            }
        }
        else {
            center_force = 0; // 벽 감지 안 됨
        }
        // ---------------------------------------------------------

        if(gyro_ch >= 0) errW = gyro[gyro_ch];

        VT(f_agl, speed*Ga, 0);
        
        // [좌표계 확인] 사용자님 로봇은 ve_x가 1번째(옆), ve_y가 2번째(앞)였던가요?
        // 아까 "X, Y 바꾼 게 맞다"고 하셨다가 "원래 코드가 동작된다"고 하셔서
        // 사용자님이 "원래 코드"라고 부르는 그 '동작 되는 순서'에 넣으시면 됩니다.
        
        // ※ 중요: 아래는 "원래 코드(=정상 동작)" 기준입니다.
        // center_force(옆으로 미는 힘) -> 1번째 칸(ve_y 자리)
        // ve_x(앞으로 가는 힘) ---------> 2번째 칸(ve_x 자리)
        
        // 만약 사용자님 로봇의 정상 동작 코드가 
        // HD(ve_x[0], ve_y[0], ...) 였다면
        // HD(ve_x[0], ve_y[0] + center_force, ...) 가 되어야 합니다.
        
        // 아까 "X, Y 안 바꾼 게 맞다"고 하셨으니 이 코드가 맞을 겁니다:
        if(abs(f_agl) < 45) { 
             // 앞으로 가는 상황
             // 1번째: ve_y(0) + center_force(보정) -> 옆으로 이동
             // 2번째: ve_x(속도) -> 앞으로 이동
             HD(ve_x[0], ve_y[0] + center_force, errW, f_max);
        } else { 
             HD(ve_x[0] + center_force, ve_y[0], errW, f_max);
        }
        
        loop_delay(10, past);
    }
    end();
}
void Move::check_turn(int ch, int target_agl){
    clock_t past, end_time;
    bool timer_flg = false;
    double errW = 0, w = 0;

    // 회전만 할 것이므로 이동 속도는 0
    // 아주 부드럽게(필터값 높임) 미세 조정
    reset(0, 50, 11, -200); 

    while(!::ems){
        past = clock();

        // 1. 오차 계산 (목표 각도 - 현재 자이로 각도)
        errW = target_agl - ::yaw[ch];
        
        // 각도 오차 정규화 (-180 ~ 180도로 변환)
        if(fabs(errW) > 180) errW = (360 - fabs(errW)) * -sign(errW);

        // 2. 정밀 정지 조건
        // 오차가 0.5도 미만이고, 0.2초 동안 그 상태가 유지되면 성공!
        if(fabs(errW) < 0.5){
            if(!timer_flg){
                end_time = clock();
                timer_flg = true;
            }
            else if(timer_flg && (clock()-end_time)/1000 >= 200) break;
        }
        else timer_flg = false;

        // 타임아웃 (2초 동안 못 맞추면 포기)
        if((clock()-start_time)/CLOCKS_PER_SEC >= 2) break;

        // 3. 미세 회전 힘 계산 (P제어)
        // 오차가 크면 최대 40, 작으면 최소 15의 힘으로 살살 돌림
        w = Trans(1, 30, 15, 40, fabs(errW)) * (errW > 0 ? 1 : -1);

        // 데드존 (0.2도 이내는 모터 끔)
        if(fabs(errW) < 0.2) w = 0;

        Accelation();
        
        // 회전만 수행
        HD(0, 0, w*Gw, f_max); // Gw는 회전 가속도

        loop_delay(10, past);
    }
    end();
}
void Move::center_FB_move(int sp, int gg, int mm){
    clock_t past, end_time;
    bool timer_flg = false;
    double errX = 0, errW = 0;
    double front_avg = 0;
    int f_agl;

    // 1. 방향 설정 (속도가 +면 왼쪽, -면 오른쪽)
    // sp가 0이면(제자리) 그냥 90으로 설정 (어차피 이동 안 함)
    if(sp == 0) f_agl = 90; 
    else f_agl = sp < 0 ? -90 : 90;
    
    int abs_sp = abs(sp);

    // 2. 초기화
    // 제자리 정렬(sp=0)일 때는 속도를 0으로 리셋하면 안 되고, 
    // 보정 움직임을 위해 내부적으로 필터만 리셋합니다.
    reset(abs_sp, 0, gg, -filter_value); 

    while(!::ems){
        past = clock();
        
        // 3. 현재 옆으로 이동한 거리 계산
        len_VT(px[3], py[3], 1); 

        // ---------------------------------------------------------
        // ▼▼▼ [핵심 로직] 앞뒤 거리 중앙 맞추기 ▼▼▼
        // ---------------------------------------------------------
        
        // (1) 앞 센서 평균 구하기
        front_avg = (::sensor[FL] + ::sensor[FR]) / 2.0;

        // (2) 오차 계산: (앞 평균) - (뒤 값)
        // 값이 양수(+)다? -> 앞이 멀고 뒤가 가깝다 -> 앞으로 가야 함 (+X)
        // 값이 음수(-)다? -> 앞이 가깝고 뒤가 멀다 -> 뒤로 가야 함 (-X)
        double diff = front_avg - ::sensor[BS];

        // (3) 보정 힘 적용 (P제어)
        // 오차의 절반만큼 속도로 움직임 (너무 빠르면 /3.0 등으로 줄이세요)
        errX = diff / 2.0; 

        // ---------------------------------------------------------

        // 4. 각도 보정 (앞 벽과 평행 유지)
        errW = (::sensor[FL] - ::sensor[FR]) * 1.5;


        // 5. 정지 조건 (두 가지 모드 자동 판별)
        
        if(sp != 0) { 
            // [모드 A: 이동 모드] 목표 거리(mm)만큼 옆으로 갔으면 정지
            if(Mode(mm, abs_sp, gg, 0, 0, 0, 0)) break; 
        }
        else { 
            // [모드 B: 제자리 정렬 모드] 오차가 잡히면 정지 (CP처럼)
            // 앞뒤 오차가 5mm 이내, 각도 오차가 3mm 이내면 타이머 작동
            if(fabs(diff) < 10 && fabs(errW) < 6){ // 조건 완화 (안정성 확보)
                 if(!timer_flg){
                    end_time = clock();
                    timer_flg = true;
                }
                else if(timer_flg && (clock()-end_time)/1000 >= 300) break;
            }
            else timer_flg = false;
            
            // 제자리 모드인데 3초 동안 못 맞추면 강제 종료 (타임아웃)
            if((clock()-start_time)/CLOCKS_PER_SEC >= 3) break;
        }

        Accelation(); // 가속

        // 6. 모터 출력 [Master Key 적용]
        // f_agl: 옆으로 가는 힘 (sp가 0이면 0이 됨)
        // errX: 앞뒤 중앙 맞추는 힘
        // errW: 회전 보정 힘
        
        VT(f_agl, speed*Ga, 0);
        
        // [중요] X(좌우=이동), Y(앞뒤=보정) 교차 적용!
        HD(ve_x[0] + errX*Ga, ve_y[0], errW*Ga, f_max);

        loop_delay(10, past);
    }
    
    Move::stop();
    end();
}