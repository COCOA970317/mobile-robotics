#pragma once

#include "Subsystems.h"
#include <functional>
#include <cmath> // std::abs 사용을 위해 추가
#include <thread> // std::this_thread 사용을 위해 추가
#include <chrono> // 시간 관련 기능

/**
 * @brief 로봇의 고수준 동작을 제어하는 클래스 (미션 수행용)
 * 주행, 회전, 벽타기 등의 복합 동작을 메서드로 제공합니다.
 */
class Navigator {
public:
    Navigator(DriveSystem& drive, SensorSystem& sensors);

    // ==========================
    // 기본 주행 명령
    // ==========================
    
    /**
     * @brief 일정 시간 동안 주행
     * @param x 좌우 속도
     * @param y 전후 속도
     * @param w 회전 속도
     * @param timeSec 지속 시간(초)
     */
    void DriveTime(double x, double y, double w, double timeSec);

    /**
     * @brief 목표 거리만큼 직진 (엔코더/시간 기반 혹은 센서 피드백 없음)
     * 이 함수보다는 센서 기반 이동을 권장합니다.
     */
    void MoveDistance(double distanceMM, double maxSpeed);

    /**
     * @brief 자이로를 이용한 제자리 회전
     * @param targetAngle 목표 각도 (상대 각도 아님, 절대 각도)
     */
    void TurnToHeading(double targetAngle);

    // ==========================
    // 센서 기반 주행 (스마트 기능)
    // ==========================

    /**
     * @brief 벽과의 거리를 유지하며 이동 (Wall Following)
     * @param sensorType 0:왼쪽초음파, 1:오른쪽초음파, 2:왼쪽PSD, 3:오른쪽PSD
     * @param targetDist 목표 거리 (mm)
     * @param speed 전진 속도
     * @param untilCondition 종료 조건 함수 (true 리턴 시 종료)
     */
    void WallFollow(int sensorType, double targetDist, double speed, std::function<bool()> untilCondition);

    /**
     * @brief 앞 벽과의 거리가 될 때까지 전진/후진
     */
    void DriveToFrontDist(double targetDist, double speed);

    /**
     * @brief 라인을 만날 때까지 이동
     */
    void DriveUntilLine(double x, double y, double speed);

    /**
     * @brief 로봇 정지 및 대기
     */
    void Stop(double waitTime = 0.0);

private:
    DriveSystem& drive;
    SensorSystem& sensors;
    
    // 내부 헬퍼 함수: PID 계산
    double CalculatePID(double error, double kp);
    void SleepMs(int ms);
};