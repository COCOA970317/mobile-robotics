#pragma once

#include <frc/Ultrasonic.h>
#include <frc/AnalogInput.h>
#include <AHRS.h>
#include <studica/Cobra.h>
#include <frc/DigitalOutput.h>

namespace Sensor{
    extern frc::AnalogInput *psd[Constants::PSD_CNT];
    extern frc::Ultrasonic *ping[Constants::PING_CNT]; 
    extern AHRS gyro;
    extern studica::Cobra cobra;
    extern frc::DigitalOutput led_g, led_r;

    void PSD_Loop();
    void Ping_Loop();
    void Gyro_Loop();
}