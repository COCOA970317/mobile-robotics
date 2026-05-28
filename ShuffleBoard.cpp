#include "All.h"
#include <frc/DriverStation.h>
#include <cameraserver/CameraServer.h>

std::shared_ptr<nt::NetworkTable> ShuffleBoard::Table;

nt::NetworkTableEntry ShuffleBoard::nt_CobraRaw[4], ShuffleBoard::nt_Cobra[4], ShuffleBoard::nt_Sensor[6];
nt::NetworkTableEntry ShuffleBoard::nt_LED_Green, ShuffleBoard::nt_LED_Red, ShuffleBoard::nt_M_cnt, ShuffleBoard::nt_Battery, ShuffleBoard::nt_Enable;
nt::NetworkTableEntry ShuffleBoard::nt_EMS, ShuffleBoard::nt_SW1, ShuffleBoard::nt_SW2, ShuffleBoard::nt_LimitSW[5];

nt::NetworkTableEntry ShuffleBoard::nt_Motor[4], ShuffleBoard::nt_Encoder[4], ShuffleBoard::nt_OMS_Z_Now;
nt::NetworkTableEntry ShuffleBoard::nt_px[4], ShuffleBoard::nt_py[4], ShuffleBoard::nt_pw[4], ShuffleBoard::nt_yaw[4];
nt::NetworkTableEntry ShuffleBoard::nt_Graph;

nt::NetworkTableEntry ShuffleBoard::nt_Test_Enable;
nt::NetworkTableEntry ShuffleBoard::nt_LED_Green_Btn, ShuffleBoard::nt_LED_Red_Btn;
nt::NetworkTableEntry ShuffleBoard::nt_Motor_Enable[4], ShuffleBoard::nt_Motor_Control[4], ShuffleBoard::nt_Servo_Enable[5], ShuffleBoard::nt_Servo_Control[5], ShuffleBoard::nt_PID[2], ShuffleBoard::nt_PID_Btn;

nt::NetworkTableEntry ShuffleBoard::nt_color_x, ShuffleBoard::nt_color_y;
nt::NetworkTableEntry ShuffleBoard::nt_Vision_Color, ShuffleBoard::nt_Vision_Barcode;
nt::NetworkTableEntry ShuffleBoard::nt_Vision_Mode;
nt::NetworkTableEntry ShuffleBoard::nt_Cnt_Green, ShuffleBoard::nt_Cnt_Orange, ShuffleBoard::nt_Cnt_Red, ShuffleBoard::nt_Cnt_Yellow;

nt::NetworkTableEntry ShuffleBoard::nt_Mission_Status;

nt::NetworkTableEntry ShuffleBoard::nt_mode;
nt::NetworkTableEntry ShuffleBoard::nt_barcode, ShuffleBoard::nt_color, ShuffleBoard::nt_ball_x, ShuffleBoard::nt_ball_count;

nt::NetworkTableEntry ShuffleBoard::nt_cnt;
nt::NetworkTableEntry ShuffleBoard::nt_now_center;
nt::NetworkTableEntry ShuffleBoard::nt_now_BaeSongGi;
nt::NetworkTableEntry ShuffleBoard::nt_complete_root;

nt::NetworkTableEntry ShuffleBoard::nt_rail_left, ShuffleBoard::nt_rail_right, ShuffleBoard::nt_detected_color;
nt::NetworkTableEntry ShuffleBoard::nt_delivery_x;
nt::NetworkTableEntry ShuffleBoard::nt_delivery_y;

std::string ShuffleBoard::Confirmed_Color = "None";

static std::string last_raw_color = "None"; 
static int match_count = 0;                 
const int LIMIT_COUNT = 15;                 

void ShuffleBoard::Update_Color_Status() {
    std::string raw_color = nt_Vision_Color.GetString("None");

    if (raw_color == last_raw_color) {
        match_count++;
    } else {
        match_count = 0;
        last_raw_color = raw_color;
    }

    if (match_count > LIMIT_COUNT) {
        Confirmed_Color = last_raw_color;
        if(match_count > 1000) match_count = LIMIT_COUNT + 1;
    }
}

std::string ShuffleBoard::Get_Barcode() {
    return nt_Vision_Barcode.GetString("None");
}

std::string ShuffleBoard::Get_Color() {
    Update_Color_Status();
    return Confirmed_Color;
}

void ShuffleBoard::Single(){
    robot_desired_mode = false;
}

void ShuffleBoard::Multi(){
    robot_desired_mode = true;
}

int ShuffleBoard::Get_Count_Red() {
    return (int)nt_Cnt_Red.GetDouble(0);
}

int ShuffleBoard::Get_Count_Orange() {
    return (int)nt_Cnt_Orange.GetDouble(0);
}

int ShuffleBoard::Get_Count_Yellow() {
    return (int)nt_Cnt_Yellow.GetDouble(0);
}

int ShuffleBoard::Get_Count_Green() {
    return (int)nt_Cnt_Green.GetDouble(0);
}

nt::NetworkTableEntry ShuffleBoard::nt_Vision_User_Switch;
bool ShuffleBoard::robot_desired_mode = false;

void ShuffleBoard::Init(){
    Table = nt::NetworkTableInstance::GetDefault().GetTable("Table");
    
    nt::NetworkTableInstance::GetDefault().SetUpdateRate(0.1);

    static cs::HttpCamera piCam("PiCamera", "http://10.21.30.2:1182/cam.mjpg");
    piCam.SetPixelFormat(cs::VideoMode::PixelFormat::kMJPEG);
    frc::CameraServer::GetInstance()->StartAutomaticCapture(piCam);

    for(int i=0;i<4;i++){
        nt_CobraRaw[i] = ShuffleBoard::Table->GetEntry("CobraRaw" + std::to_string(i));
        nt_Cobra[i] = ShuffleBoard::Table->GetEntry("Cobra" + std::to_string(i));
    }
    for(int i=0;i<6;i++){
        nt_Sensor[i] = ShuffleBoard::Table->GetEntry("Sensor" + std::to_string(i));
    }
    nt_LED_Green = ShuffleBoard::Table->GetEntry("LED_Green");
    nt_LED_Red = ShuffleBoard::Table->GetEntry("LED_Red");
    nt_M_cnt = ShuffleBoard::Table->GetEntry("M_cnt");
    nt_Battery = ShuffleBoard::Table->GetEntry("Battery");
    nt_Enable = ShuffleBoard::Table->GetEntry("Enable");

    nt_EMS = ShuffleBoard::Table->GetEntry("EMS");
    nt_SW1 = ShuffleBoard::Table->GetEntry("SW1");
    nt_SW2 = ShuffleBoard::Table->GetEntry("SW2");
    
    // 5개의 리밋 스위치를 네트워크 테이블에 등록
    for(int i=0; i<5; i++){
        nt_LimitSW[i] = ShuffleBoard::Table->GetEntry("LimitSW" + std::to_string(i));
    }

    for(int i=0;i<4;i++){
        nt_Motor[i] = ShuffleBoard::Table->GetEntry("Motor" + std::to_string(i));
        nt_Encoder[i] = ShuffleBoard::Table->GetEntry("Encoder" + std::to_string(i));

        nt_px[i] = ShuffleBoard::Table->GetEntry("px" + std::to_string(i));
        nt_py[i] = ShuffleBoard::Table->GetEntry("py" + std::to_string(i));
        nt_pw[i] = ShuffleBoard::Table->GetEntry("pw" + std::to_string(i));
        nt_yaw[i] = ShuffleBoard::Table->GetEntry("yaw" + std::to_string(i));
    }
    nt_OMS_Z_Now = ShuffleBoard::Table->GetEntry("OMS_Z_Now");
    nt_Graph = ShuffleBoard::Table->GetEntry("Graph");
    
    nt_Test_Enable = ShuffleBoard::Table->GetEntry("Test_Enable");
    nt_LED_Green_Btn = ShuffleBoard::Table->GetEntry("LED_G_Btn");
    nt_LED_Red_Btn = ShuffleBoard::Table->GetEntry("LED_R_Btn");
    for(int i=0;i<4;i++){
        nt_Motor_Enable[i] = ShuffleBoard::Table->GetEntry("Motor_Enable" + std::to_string(i));
        nt_Motor_Control[i] = ShuffleBoard::Table->GetEntry("Motor_Control" + std::to_string(i));
    }
    for(int i=0;i<5;i++){
        nt_Servo_Enable[i] = ShuffleBoard::Table->GetEntry("Servo_Enable" + std::to_string(i));
        nt_Servo_Control[i] = ShuffleBoard::Table->GetEntry("Servo_Control" + std::to_string(i));
    }
    nt_PID[0] = ShuffleBoard::Table->GetEntry("P_Control");
    nt_PID[1] = ShuffleBoard::Table->GetEntry("I_Control");
    nt_PID_Btn = ShuffleBoard::Table->GetEntry("PID_Btn");

    //==================================================================================
    // Vision
    nt_mode = ShuffleBoard::Table->GetEntry("mode");
    nt_barcode = ShuffleBoard::Table->GetEntry("barcode");
    nt_color = ShuffleBoard::Table->GetEntry("color");
    nt_ball_x = ShuffleBoard::Table->GetEntry("ball_x");
    nt_ball_count = ShuffleBoard::Table->GetEntry("ball_count");

    nt_cnt = ShuffleBoard::Table->GetEntry("cnt");

    nt_rail_left = ShuffleBoard::Table->GetEntry("rail_left_str");
    nt_rail_right = ShuffleBoard::Table->GetEntry("rail_right_str");

    nt_detected_color = ShuffleBoard::Table->GetEntry("detected_color");

    nt_delivery_x = ShuffleBoard::Table->GetEntry("delivery_x");
    nt_delivery_y = ShuffleBoard::Table->GetEntry("delivery_y");
    
    // 2. 초기값 세팅하기 (Init 함수 맨 아래)
    ShuffleBoard::nt_detected_color.SetString("None");

    //==================================================================================

    nt_color_x = ShuffleBoard::Table->GetEntry("color_x");
    nt_color_y = ShuffleBoard::Table->GetEntry("color_y");

    nt_Vision_Color = ShuffleBoard::Table->GetEntry("Vision_Color");
    nt_Vision_Barcode = ShuffleBoard::Table->GetEntry("Vision_Barcode");

    nt_Cnt_Red = ShuffleBoard::Table->GetEntry("Count_Red");
    nt_Cnt_Red.SetDouble(0);

    nt_Cnt_Orange = ShuffleBoard::Table->GetEntry("Count_Orange");
    nt_Cnt_Orange.SetDouble(0);

    nt_Cnt_Yellow = ShuffleBoard::Table->GetEntry("Count_Yellow");
    nt_Cnt_Yellow.SetDouble(0);

    nt_Cnt_Green = ShuffleBoard::Table->GetEntry("Count_Green");
    nt_Cnt_Green.SetDouble(0);

    nt_Vision_Mode = ShuffleBoard::Table->GetEntry("Vision_Mode");
    nt_Vision_Mode.SetBoolean(false);

    nt_Vision_User_Switch = Table->GetEntry("Vision_User_Override");
    nt_Vision_User_Switch.SetBoolean(false);

    nt_Mission_Status = ShuffleBoard::Table->GetEntry("Mission_Status");
    nt_Mission_Status.SetString("Ready");
    
    delay(1000,false);
    ShuffleBoard::nt_Enable.SetBoolean(true);
    ShuffleBoard::nt_Test_Enable.SetBoolean(false);

    ShuffleBoard::nt_LED_Green_Btn.SetBoolean(false);
    ShuffleBoard::nt_LED_Red_Btn.SetBoolean(false);

    for(int i=0;i<4;i++){
        ShuffleBoard::nt_Motor_Enable[i].SetBoolean(false);
        ShuffleBoard::nt_Motor_Control[i].SetDouble(0);
    }
    for(int i=0;i<5;i++){
        ShuffleBoard::nt_Servo_Enable[i].SetBoolean(false);
        ShuffleBoard::nt_Servo_Control[i].SetDouble(150);
    }
    nt_PID[0].SetDouble(Constants::OMS_PID_K[0]);
    nt_PID[1].SetDouble(Constants::OMS_PID_K[1]);
    nt_PID_Btn.SetBoolean(0);

    ShuffleBoard::nt_mode.SetDouble(0);
    ShuffleBoard::nt_barcode.SetString("");
    ShuffleBoard::nt_color.SetDouble(-1);
    ShuffleBoard::nt_ball_x.SetDouble(0);
    ShuffleBoard::nt_ball_count.SetDouble(0);
    ShuffleBoard::nt_cnt.SetDouble(0);

    ShuffleBoard::nt_rail_left.SetString(""); 
    ShuffleBoard::nt_rail_right.SetString("");

    ShuffleBoard::nt_delivery_x.SetDouble(0.0);
    ShuffleBoard::nt_delivery_y.SetDouble(0.0);
}

void ShuffleBoard::Loop(){
    clock_t past;
    int num;

    Init();
    while(true){
        past = clock();

        Update_Color_Status();

        bool user_want = nt_Vision_User_Switch.GetBoolean(false);
        bool final_mode = robot_desired_mode || user_want;

        nt_Vision_Mode.SetBoolean(final_mode);

        nt_Battery.SetDouble(frc::DriverStation::GetInstance().GetBatteryVoltage());

        num=0;
        for(int i=0;i<4;i++){
            cobra_raw[i]=Sensor::cobra.GetRawValue(i);
            num+=(cobra_raw[i]>=Constants::COBRA_SENSITIVITY) << i;
        }
        ::cobra = num;

        Sensor::led_g.Set(::led_g);
        Sensor::led_r.Set(::led_r);

        if(ShuffleBoard::nt_Enable.GetBoolean(false)){
            for(int i=0;i<4;i++){
                ShuffleBoard::nt_CobraRaw[i].SetDouble(::cobra_raw[i]);
                ShuffleBoard::nt_Cobra[i].SetBoolean(::cobra & (1<<i));
            }
            
            for(int i=0;i<6;i++){
                ShuffleBoard::nt_Sensor[i].SetDouble(::sensor[i]);
            }

            ShuffleBoard::nt_LED_Green.SetBoolean(::led_g);
            ShuffleBoard::nt_LED_Red.SetBoolean(::led_r);

            ShuffleBoard::nt_M_cnt.SetDouble(::M_cnt);
            ShuffleBoard::nt_Battery.SetDouble(frc::DriverStation::GetInstance().GetBatteryVoltage());

            ShuffleBoard::nt_EMS.SetBoolean(::ems);
            ShuffleBoard::nt_SW1.SetBoolean(::sw1);
            ShuffleBoard::nt_SW2.SetBoolean(::sw2);

            // 5개 리밋 스위치 상태 업데이트
            for(int i=0; i<5; i++){
                ShuffleBoard::nt_LimitSW[i].SetBoolean(LimitSW::is_holding(i));
            }

            for(int i=0;i<4;i++){
                ShuffleBoard::nt_Motor[i].SetDouble(Motor::motor[i].Get());
                ShuffleBoard::nt_Encoder[i].SetDouble(::encoder[i]);

                ShuffleBoard::nt_px[i].SetDouble(::px[i]);
                ShuffleBoard::nt_py[i].SetDouble(::py[i]);
                ShuffleBoard::nt_pw[i].SetDouble(::pw[i]);
                ShuffleBoard::nt_yaw[i].SetDouble(::yaw[i]);
            }
            ShuffleBoard::nt_OMS_Z_Now.SetDouble(::oms_z_now);
        }

        ::Disable=ShuffleBoard::nt_Test_Enable.GetBoolean(false);

        if(!::ems && ::Disable){
            ::led_g = ShuffleBoard::nt_LED_Green_Btn.GetBoolean(false);
            ::led_r = ShuffleBoard::nt_LED_Red_Btn.GetBoolean(false);
            for(int i=0;i<4;i++){
                if(!ShuffleBoard::nt_Motor_Enable[i].GetBoolean(false)) ShuffleBoard::nt_Motor_Control[i].SetDouble(0);
                ::motor[i]=ShuffleBoard::nt_Motor_Control[i].GetDouble(false);
            }
            for(int i=0;i<Constants::SERVO_CNT;i++){
                if(ShuffleBoard::nt_Servo_Enable[i].GetBoolean(false)) ::servo[i]=ShuffleBoard::nt_Servo_Control[i].GetDouble(false);
                else ::servo[i]=-1;
            }
        }
        
        if(clock()-past>=50000) cout << "ShuffleBoard Timeout: " << clock()-past << endl;

        delay(30 - (clock()-past)/1000, false);
    }
}