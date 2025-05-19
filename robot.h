#pragma once

#include "chassis.h"
#include <openmv.h>
#include <servo32u4.h>
#include <UltrasonicSensor.h>

class Robot
{
protected:
    /**
     * We define some modes for you. SETUP is used for adjusting gains and so forth. Most
     * of the activities will run in AUTO. You shouldn't need to mess with these.
     */
    enum ROBOT_CTRL_MODE
    {
        CTRL_TELEOP,
        CTRL_AUTO,
    };
    ROBOT_CTRL_MODE robotCtrlMode = CTRL_AUTO;

    /**
     * robotState is used to track the current task of the robot. You will add new states as 
     * the term progresses.
     */
    enum ROBOT_STATE 
    {
        ROBOT_IDLE,
        ROBOT_DRIVE_TO_POINT,
        ROBOT_PATH_FOLLOWING,
        ROBOT_SEARCHING,
        ROBOT_APPROACHING,
        ROBOT_TRAIN,
        ROBOT_SPINNING,
        ROBOT_TURN180,
        ROBOT_SENSING_TEST_ULTRASONIC,
        ROBOT_LIFT,
        ROBOT_DROP_OFF,
    };
    ROBOT_STATE robotState = ROBOT_IDLE;

    /* Define the chassis*/
    Chassis chassis;

    // For managing key presses
    String keyString;

    /**
     * For tracking current pose and the destination.
     */
    Pose destPose;
    Pose prevPose;
    Pose currPose;
    
    OpenMV camera;
    AprilTagDatum tag;

    //Servo
    Servo32U4Pin5 servo5;

    //Ultrasonic
    float ultrasonicDistance = 0;
    UltrasonicSensor ultrasonicSensor;
    
public:
    
    Robot(void) {keyString.reserve(10);}
    void InitializeRobot(void);
    void RobotLoop(void);
    bool CheckSerialInput(void);
    void ParseSerialInput(void);

    long prevMicro = 0;
    float d = 3.0;
    float maxSpeedPath = (29.0/12.0) * d;
    float alignDistance = 22;
    int wantedTagID = 0;
    int dropOffTagID = 5;
    bool needPicture = true;
    int servoPose = 1450;
    float distanceOffset = 10;

protected:
    
    /* For managing IR remote key presses*/
    void HandleKeyCode(int16_t keyCode);

    /* State changes */    
    void EnterIdleState(void);
    void EnterSearchingState(Pose, int);
    void EnterTrainState(int);
    void EnterApproach(int);
    void EnterDriveToPoint(Pose);
    void EnterSpinningState(void);
    void EnterSensingTestUltraSonic(void);
    void EnterTurn180(void);
    void EnterLift(void);
    void EnterDropoff(void);
    void EnterTimeTurn(void);

    /* Mode changes */
    void EnterTeleopMode(void);
    void EnterAutoMode(void);

    // /* Navigation methods.*/
    void UpdatePose(const Twist& u);
    void SetDestination(const Pose& destination);
    void UpdateDestPoint(Pose, Pose);
    void DriveToPoint(Pose);
    bool CheckReachedDestination(void);
    void HandleDestination(void);
    void HandleTag(void);
    void ApproachTag(void);
    void AlignToTag(void);
    bool CheckAligned(void);
    bool CheckApproachDone(void);
    void HandleApproachDone(void);
    void HandleDistanceReadingUltrasonic(float);
    bool CheckLiftComplete(void);
};
