/**
 * robot-nav.cpp is where you should put navigation routines.
 */

#include "robot.h"

void Robot::UpdatePose(const Twist& twist)
{
    long currMicro = micros();
    float deltaT = (currMicro - prevMicro) / 1000000.0;

    //First order approximation
    currPose.x = prevPose.x - deltaT * twist.u * sin(currPose.theta);
    currPose.y = prevPose.y + deltaT * twist.u * cos(currPose.theta);
    currPose.theta = (prevPose.theta + (deltaT * twist.omega / 2.0)); // 2 wheels, compensate for that in formla
    if(currPose.theta > PI) currPose.theta -= 2 * PI;
    if(currPose.theta < -PI) currPose.theta += 2 * PI;

    prevPose.x = currPose.x;
    prevPose.y = currPose.y;
    prevPose.theta = currPose.theta;
    prevMicro = currMicro;

    #ifdef __NAV_DEBUG__
        TeleplotPrint("x", currPose.x);
        TeleplotPrint("y", currPose.y);
        TeleplotPrint("theta", currPose.theta * 180/PI);
    #endif
}

/**
 * Sets a destination in the lab frame.
 */
void Robot::SetDestination(const Pose& dest)
{
    //digitalWrite(13, HIGH); // Turn on the LED (I dont remember the LED Pin)

    // Serial.print("destX: ");
    // Serial.print(dest.x);
    // Serial.print(" | destY: ");
    // Serial.print(dest.y);
    // Serial.print(" | destθ: ");
    // Serial.println(dest.theta * 180/PI);

    destPose = dest;
}

bool Robot::CheckReachedDestination(void)
{
    //Serial.print("");
    float dx = destPose.x - currPose.x;
    float dy = destPose.y - currPose.y;
    float distance = sqrt((dx * dx) + (dy * dy));
    float angularDistance = abs(destPose.theta - currPose.theta);
    
    // Serial.println(currPose.theta);
    // Serial.println(distance);
    return distance < 2 && angularDistance < 0.02;
}


void Robot::DriveToPoint(Pose Pose)
{   if (robotState == ROBOT_PATH_FOLLOWING){
        float dx = currPose.x - Pose.x;
        float dy = Pose.y - currPose.y;
        float distance = sqrt((dx * dx) + (dy * dy));
        float absDY = abs(destPose.y - currPose.y);

        // Desired heading
        float desiredTheta = atan2(dx, dy);

        // Heading error (normalized)
        float headingError = desiredTheta - currPose.theta;
        if(headingError > PI) headingError -= 2 * PI;
        if(headingError < -PI) headingError += 2 * PI;

        // Controller gains (This may need to be editted)
        float kLinear = 1;
        float kAngular = 0.65;

        // Slow down as we get close
        float linearSpeed = absDY * kLinear;
        float angularSpeed = kAngular * headingError;
        // Scale linear speed based on orientation to target position 
        linearSpeed = ((PI/2.0) - abs(headingError)) * linearSpeed;

        // limit linear speed
        linearSpeed = min(max(0, linearSpeed), maxSpeedPath);
        Serial.print("speed: ");
        Serial.print(linearSpeed);
        Serial.print(" | distance: ");
        Serial.println(distance);


        chassis.SetTwist(Twist(linearSpeed, 0, angularSpeed));
    }
    if (robotState == ROBOT_DRIVE_TO_POINT || ROBOT_SEARCHING){
        // Compute the vector to the destination
        float dx = currPose.x - Pose.x;
        float dy = Pose.y - currPose.y;
        float distance = sqrt((dx * dx) + (dy * dy));

        // Desired heading
        float desiredTheta = atan2(dx, dy);

        // Heading error (normalized)
        float headingError = desiredTheta - currPose.theta;
        if(headingError > PI) headingError -= 2 * PI;
        if(headingError < -PI) headingError += 2 * PI;

        // Controller gains (This may need to be editted)
        float kLinear = 0.5;
        float kAngular = 0.65;

        // Slow down as we get close
        float linearSpeed = kLinear * distance;
        float angularSpeed = kAngular * headingError;

        //// Scale linear speed based on orientation to target position 
        //linearSpeed = ((PI/2.0) - abs(headingError)) * linearSpeed;

        // limit linear speed
        float maxSpeed = 15; // cm/s
        linearSpeed = min(max(0, linearSpeed), maxSpeed);
        
        if (abs(headingError) > PI/2){
            linearSpeed *= -1;
            angularSpeed = -kAngular * (abs(headingError) - PI);
        }
        
        float dTheta = Pose.theta - currPose.theta;
        float kAngularError = 0.65;
        float angularError = dTheta * kAngularError;
        float kProportion = min(1, max(0, 5 - (2 * distance)));
        
        // Serial.print("distance: ");
        // Serial.print(distance);
        // Serial.print(" | angular speed: ");
        // Serial.print(angularSpeed);
        // Serial.print(" | angular ERROR: ");
        // Serial.print(angularError);
        // Serial.print(" | kProportion: ");
        // Serial.print(kProportion);
        // Serial.print(" | Current Theta: ");
        // Serial.println(currPose.theta);
        
        // Serial.print("Delta_X: ");
        // Serial.print(dx);
        // Serial.print(" | Delta_Y: ");
        // Serial.print(dy);
        // Serial.print(" | Pose_X: ");
        // Serial.print(currPose.x);
        // Serial.print(" | Pose_Y: ");
        // Serial.println(currPose.y);

        chassis.SetTwist(Twist(linearSpeed, 0, (angularSpeed * (1 - kProportion)) + (angularError * kProportion)));
        // if (distance > 1.5) {
        //     chassis.SetTwist(Twist(linearSpeed, 0, angularSpeed));//(angularSpeed * (1 - kProportion)) + (angularError * kProportion)));
        // } else {
        //     chassis.SetTwist(Twist(linearSpeed, 0, (angularError * kProportion )));
        // }
    #ifdef __NAV_DEBUG__
            TeleplotPrint("dist", distance);
            TeleplotPrint("hdgErr", headingError);
            TeleplotPrint("spd", linearSpeed);
    #endif
    }
}

bool Robot::CheckAligned(){
    if (wantedTagID == 5){
        int tagX = tag.xSign ? tag.x : -tag.x;
        float tagAngle = tag.angle * (PI/180.0);
        if(tag.angleSign) tagAngle *= -1;
        bool xAligned = abs(tagX) < 6; // 4 cm of tolerance
        bool yAligned = abs((alignDistance + distanceOffset) - (int)tag.y) < 7; // 7 cm of tolerance
        bool rotAligned = abs(tagAngle) < PI * (15) / 180.0;
        
        Serial.print("tagX:");
        Serial.print(abs(tagX));
        Serial.print(" | tagY:");
        Serial.print(abs(alignDistance - (int)tag.y)) && rotAligned;

        return xAligned && yAligned;
    } else {
        int tagX = tag.xSign ? tag.x : -tag.x;
        float tagAngle = tag.angle * (PI/180.0);
        if(tag.angleSign) tagAngle *= -1;
        bool xAligned = abs(tagX) < 2; // 4 cm of tolerance
        bool yAligned = abs(alignDistance - (int)tag.y) < 7; // 7 cm of tolerance
        bool rotAligned = abs(tagAngle) < PI * (10) / 180.0;
        
        Serial.print("tagX:");
        Serial.print(abs(tagX));
        Serial.print(" | tagY:");
        Serial.print(abs(alignDistance - (int)tag.y));

        return xAligned && yAligned && rotAligned;
    }
}

void Robot::HandleDestination(void)
{
    chassis.Stop();
    Serial.println("Reached destination.");
    switch(robotState){
    case ROBOT_SEARCHING:
        if (CheckAligned()) {
            Serial.println("Aligned.");
            if (wantedTagID == dropOffTagID) EnterDropoff();
            else EnterTurn180();
        
        } else {
            Serial.println("Not aligned.");
            needPicture = true;
            EnterSpinningState();
        }
        break;
    case ROBOT_DRIVE_TO_POINT:
        EnterIdleState();
        break;
    case ROBOT_TURN180:
        EnterApproach(wantedTagID);
        break;
    case ROBOT_DROP_OFF:
        EnterDriveToPoint(Pose(0, 30, 0));
}}

void Robot::HandleTag()
{
    if (tag.id != wantedTagID) return;
    if (!needPicture) return;
    switch(robotState){
    case ROBOT_SPINNING:
        chassis.Stop();
        EnterSearchingState(currPose, wantedTagID);
    case ROBOT_SEARCHING:
        //if (tag.x > 6) return;
        float distance_offset = 0;
        if (tag.id == 5) distance_offset = distanceOffset;

        needPicture = false;
        int tagX = tag.xSign ? tag.x : -tag.x;
        float tagAngle = tag.angle * (PI/180.0);
        if(tag.angleSign) tagAngle *= -1;

        float a = (tagX + ((alignDistance + distance_offset) * sin(tagAngle)));
        float b = (tag.y - ((alignDistance + distance_offset) * cos(tagAngle)));
        float c = sqrt((a*a) + (b*b));
        float alpha = atan2f(a, b);
        float chi = alpha - currPose.theta;

        float deltaX = c * sin(chi);
        float deltaY = c * cos(chi);
        float destinationX = currPose.x + deltaX;
        float destinationY = currPose.y + deltaY;
        float destinationTheta = currPose.theta + tagAngle;

        SetDestination(Pose {destinationX, destinationY, destinationTheta});

        // Serial.print("CurrX: ");
        // Serial.print(currPose.x);
        // Serial.print(" | CurrY: ");
        // Serial.print(currPose.y);
        // Serial.print(" | Currθ: ");
        // Serial.print(currPose.theta * 180/PI);

        // Serial.print(" | tagX: ");
        // Serial.print(tagX);
        // Serial.print(" | tagY: ");
        // Serial.print(tag.y);
        // Serial.print(" | tagθ: ");
        // Serial.print(tagAngle * 180/PI);

        // Serial.print(" | theta: ");
        // Serial.print(currPose.theta * 180/PI);
        // Serial.print(" | chi: ");
        // Serial.print(chi * 180/PI);
        // Serial.print(" | alpha: ");
        // Serial.print(alpha * 180/PI);

        // Serial.print(" | destX: ");
        // Serial.print(destinationX);
        // Serial.print(" | destY: ");
        // Serial.print(destinationY);
        // Serial.print(" | destθ: ");
        // Serial.println(destinationTheta * 180/PI);
        break;
    default:
        break;
}}

const float totalDistance = 16.5; // cm
bool Robot::CheckApproachDone()
{switch(robotState){
    case ROBOT_APPROACHING:
        Serial.print("distanceY: ");
        Serial.print(tag.y);
        Serial.print(" | distanceX: ");
        Serial.println(tag.x);
        return tag.y <= totalDistance;
        break;
    case ROBOT_TRAIN:
        return (tag.y <= 20);
        break;
    default:
        return false;
        break;
}}

void Robot::HandleApproachDone()
{switch(robotState){
    case ROBOT_TRAIN:
        chassis.SetTwist(Twist(0, 0, 0));
        break;
    default:
        break;
}}

void Robot::ApproachTag()
{
    float linearSpeed;
    float angularSpeed;
    switch(robotState){
    case ROBOT_APPROACHING:
        linearSpeed = -0.35 * ultrasonicDistance;
        linearSpeed = constrain(linearSpeed, -1, -7.5);
        angularSpeed = 0;
    case ROBOT_TRAIN:
        int tagX = tag.xSign ? tag.x : -tag.x;
        float tagAngle = tag.angle * (PI/180.0);
        if(tag.angleSign) tagAngle *= -1;

        linearSpeed = 0.5 * (tag.y - totalDistance);
        angularSpeed = -0.08 * tagX;

        linearSpeed = constrain(linearSpeed, 1, 15);
        angularSpeed = constrain(angularSpeed, -0.75, 0.75);
        break;
    chassis.SetTwist(Twist(linearSpeed, 0, angularSpeed));
}}

bool scanStart = false;
bool isObjectNear = false;
void Robot::HandleDistanceReadingUltrasonic(float distance)
{
ultrasonicSensor.handleGetReading();
switch (robotState) {
case ROBOT_SENSING_TEST_ULTRASONIC:
    //distance = ultrasonicSensor.MedianFilter(distance);
    Serial.println(distance);
    break;
case ROBOT_APPROACHING:
    if (distance <= 3.5) EnterLift();
    break;
default: 
    break;
}}

bool Robot::CheckLiftComplete(){
    return servo5.currentPos < 550;
}