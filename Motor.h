#pragma once

#include <studica/TitanQuad.h>
#include <frc/Encoder.h>
#include <studica/Servo.h>
#include <frc/controller/PIDController.h>

namespace Motor{
    extern studica::TitanQuad motor[4];
    extern frc::Encoder encoder[4];
    extern studica::Servo *servo[Constants::SERVO_CNT];
    extern frc2::PIDController pid[4];

    void Reset(int &step); // 상하축 초기화

    int Find_Titan_Number(int num); // Titan 번호 찾기
    void Init();
    void Loop();
}