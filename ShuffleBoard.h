#pragma once
#include <cameraserver/CameraServer.h>
#include <frc/shuffleboard/Shuffleboard.h>

namespace ShuffleBoard{
    // 셔플보드 변수
    extern std::shared_ptr<nt::NetworkTable> Table;
    extern nt::NetworkTableEntry nt_CobraRaw[4], nt_Cobra[4], nt_Sensor[6];
    extern nt::NetworkTableEntry nt_LED_Green, nt_LED_Red, nt_M_cnt, nt_Battery, nt_Enable;
    extern nt::NetworkTableEntry nt_EMS, nt_SW1, nt_SW2, nt_LimitSW[5];

    extern nt::NetworkTableEntry nt_Motor[4], nt_Encoder[4], nt_OMS_Z_Now;
    extern nt::NetworkTableEntry nt_px[4], nt_py[4], nt_pw[4], nt_yaw[4];
    extern nt::NetworkTableEntry nt_Graph;

    extern nt::NetworkTableEntry nt_Test_Enable;
    extern nt::NetworkTableEntry nt_LED_Green_Btn, nt_LED_Red_Btn;
    extern nt::NetworkTableEntry nt_Motor_Enable[4], nt_Motor_Control[4], nt_Servo_Enable[5], nt_Servo_Control[5], nt_PID[2], nt_PID_Btn;

    extern nt::NetworkTableEntry nt_cnt;
    extern nt::NetworkTableEntry nt_now_center;
    extern nt::NetworkTableEntry nt_now_BaeSongGi;
    extern nt::NetworkTableEntry nt_complete_root;

    void Init();
    void Loop();

    // Vision
    extern nt::NetworkTableEntry nt_color_x, nt_color_y;

    extern nt::NetworkTableEntry nt_Vision_Color;
    extern nt::NetworkTableEntry nt_Vision_Barcode;

    extern nt::NetworkTableEntry nt_Vision_Mode;
    
    extern nt::NetworkTableEntry nt_Vision_User_Switch;
    extern bool robot_desired_mode;

    extern nt::NetworkTableEntry nt_Cnt_Red, nt_Cnt_Orange, nt_Cnt_Yellow, nt_Cnt_Green;

    extern nt::NetworkTableEntry nt_Mission_Status;

    extern std::string Confirmed_Color;
    std::string Get_Color();
    std::string Get_Barcode();
    void Update_Color_Status();

    void Single();
    void Multi();

    int Get_Count_Red();
    int Get_Count_Orange();
    int Get_Count_Yellow();
    int Get_Count_Green();

    extern nt::NetworkTableEntry nt_mode, nt_barcode, nt_color, nt_ball_x, nt_ball_count;
    extern nt::NetworkTableEntry nt_rail_left, nt_rail_right;
    extern nt::NetworkTableEntry nt_detected_color;
    extern nt::NetworkTableEntry nt_delivery_x; 
    extern nt::NetworkTableEntry nt_delivery_y;
}