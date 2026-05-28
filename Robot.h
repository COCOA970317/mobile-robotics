/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <atomic>
#include <frc/RobotBase.h>

#include "All.h"
#include "Mission.h"

class Robot : public frc::RobotBase {
public:
    void StartCompetition() override;
    void EndCompetition() override;
private:
    std::atomic<bool> m_exit{false};
};
