#include "robot.h"
#include <IRdecoder.h>
#include <HC-SR04.h>
HC_SR04 hc_sr04(2, 18);
void ISR_HC_SR04(void) {hc_sr04.ISR_echo();}

void Robot::InitializeRobot(void)
{
    chassis.InititalizeChassis();
    
    /**
     * Initialize the IR decoder. Declared extern in IRdecoder.h; see robot-remote.cpp
     * for instantiation and setting the pin.
     */
    decoder.init();
    
    destPose = {0, 0, 0};
    currPose = {0, 0, 0};
    prevPose = {0, 0, 0};
    prevMicro = micros();
    
    needPicture = false;

    Twist velocity(0, 0, 0);

    chassis.SetTwist(velocity);
    /*
    Serial.print(currPose.x);
    Serial.print(" | ");
    Serial.print(currPose.y);
    Serial.print(" | ");
    Serial.println(currPose.theta);
    */
    ultrasonicSensor.Initialize();
    hc_sr04.init(ISR_HC_SR04);
    pinMode(13, OUTPUT);
    pinMode(22, INPUT);
    pinMode(12, OUTPUT);
    servo5.attach();
    servo5.liftServo();
    EnterIdleState();
}

void Robot::EnterIdleState(void)
{
    chassis.Stop();
    Serial.println("-> IDLE");
    robotState = ROBOT_IDLE;
}

void Robot::EnterDriveToPoint(Pose destination)
{
    Serial.println("-> Drivinng to Point");
    needPicture = false;
    SetDestination(destination);
    robotState = ROBOT_DRIVE_TO_POINT;
}

void Robot::EnterSearchingState(Pose destination, int tagID)
{
    Serial.println("-> Searching");
    needPicture = true;
    wantedTagID = tagID;
    SetDestination(destination);
    robotState = ROBOT_SEARCHING;
}

void Robot::EnterApproach(int tagID)
{
    Serial.println("-> Approaching");
    wantedTagID = tagID;
    robotState = ROBOT_APPROACHING;
}

void Robot::EnterTrainState(int tagID)
{
    Serial.println("-> Train");
    wantedTagID = tagID;
    robotState = ROBOT_TRAIN;
}

void Robot::EnterSpinningState()
{
    Serial.println("-> Spinning");
    needPicture = true;
    robotState = ROBOT_SPINNING;
}

void Robot::EnterTurn180()
{
    Serial.println("-> Turnning");
    servo5.dropServo();
    float newTheta;
    int tagX = tag.xSign ? tag.x : -tag.x;
    float thetaOffset = atan2(tagX, alignDistance);
    float currTheta = currPose.theta - (1 * thetaOffset);
    if(currTheta < 0) newTheta = PI + currTheta;
    else newTheta = currTheta - PI;
    SetDestination(Pose(currPose.x, currPose.y, newTheta));
    robotState = ROBOT_TURN180;
}

void Robot::EnterSensingTestUltraSonic(void) 
{
    Serial.println("-> Sensing Test Ultrasonic");
    ultrasonicSensor.Initialize();
    robotState = ROBOT_SENSING_TEST_ULTRASONIC;
}

void Robot::EnterLift(void){
    Serial.println("-> Lifting");
    servo5.liftServo();
    robotState = ROBOT_LIFT;
}

void Robot::EnterDropoff(void)
{
    Serial.println("-> Dropping");
    servo5.dropServo();
    SetDestination(Pose(currPose.x, currPose.y, 0));
    robotState = ROBOT_DROP_OFF;
}

/**
 * The main loop for your robot. Process both synchronous events (motor control),
 * and asynchronous events (IR presses, distance readings, etc.).
*/
void Robot::RobotLoop(void) 
{
    /**
     * Handle any IR remote keypresses.
     */
    int16_t keyCode = decoder.getKeyCode();
    if(keyCode != -1) HandleKeyCode(keyCode);

    /**
     * Run the chassis loop, which handles low-level control.
     */
    Twist velocity;
    if(chassis.ChassisLoop(velocity))
    {
        // We do FK regardless of state
        UpdatePose(velocity);
        servo5.update();

        /**
         * Here, we break with tradition and only call these functions if we're in the 
         * DRIVE_TO_POINT state. CheckReachedDestination() is expensive, so we don't want
         * to do all the maths when we don't need to.
         * 
         * While we're at it, we'll toss DriveToPoint() in, as well.
         */
        switch(robotState){
            case ROBOT_DRIVE_TO_POINT:
            case ROBOT_SEARCHING:
            case ROBOT_DROP_OFF:
            case ROBOT_TURN180:
                DriveToPoint(destPose);
                if(CheckReachedDestination()) HandleDestination();
                break;
            case ROBOT_SPINNING:
                chassis.SetTwist(Twist(0, 0, 0.5));
                break;
            case ROBOT_APPROACHING:
                ApproachTag();
                chassis.SetTwist(Twist(-2.5, 0, 0));
                break;
            case ROBOT_TRAIN:
                ApproachTag();
                if (CheckApproachDone()) HandleApproachDone();
                break;
            case ROBOT_LIFT:
                if(CheckLiftComplete()) EnterSearchingState(Pose(0, 30, -PI/2), 5);
                break;
            case ROBOT_PATH_FOLLOWING:
                float a = 30;
                float l = destPose.y;
                // slope of function
                float s = (2 * PI * a / l) * cos(currPose.y * 2 * PI / l);
                float point_y = currPose.y + sqrt((d * d) / ((s * s) + 1));
                float point_x = a * sin(point_y * 2 * PI / l);
                DriveToPoint(Pose(point_x, point_y, 0));
                if((destPose.y - currPose.y) <= 0.5) HandleDestination();
                break;
            default:
                break;
        }

        bool newTag = camera.checkUART(tag);
        if (newTag) HandleTag();
        digitalWrite(12, newTag);
        digitalWrite(13, analogRead(22) > 600);

        if(hc_sr04.getDistance(ultrasonicDistance)) HandleDistanceReadingUltrasonic(ultrasonicDistance);
    }
}