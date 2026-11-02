#include <cmath>
#include "Arduino.h"
#include "Enes100.h"

const float ANGLE_TOLERANCE = 2.5 * PI / 180; // max radians from target to consider a turn() complete at`
const float TARG_DIST_TOLERANCE = 0.05; // max meters from target to consider a move() complete at
const float OBSTACLE_DIST_TOLERANCE = 0.2; // max meters from obstacle to consider a move() failed at
const float TURNING_BASE_PWR = 63; // power/motor to use during a turn() for every radian away from the target angle
const float TURNING_DAMPING_FACTOR = 0.05;
const float MOVEMENT_BASE_PWR = 127; // power/motor to use during a move() for every meter away from the target position
const float MOVEMENT_DAMPING_FACTOR = 0.05;
const unsigned long WAIT_TIME = 10;
const unsigned char LEFT_MOTOR_BACK_PIN_NUM = 6;
const unsigned char LEFT_MOTOR_FORTH_PIN_NUM = 7;
const unsigned char RIGHT_MOTOR_BACK_PIN_NUM = 8;
const unsigned char RIGHT_MOTOR_FORTH_PIN_NUM = 9;
const unsigned char ULTRASONIC_SENSOR_TRIG_PIN_NUM = 10;
const unsigned char ULTRASONIC_SENSOR_ECHO_PIN_NUM = 11;
const int ARUCO_MARKER_ID = 3;

// apply motor power values
void maneuver(int leftPwr, int rightPwr) {
	leftPwr = constrain(leftPwr, -255, 255);
	rightPwr = constrain(rightPwr, -255, 255);
	analogWrite(LEFT_MOTOR_BACK_PIN_NUM, leftPwr < 0 ? -leftPwr : 0);
	analogWrite(LEFT_MOTOR_FORTH_PIN_NUM, leftPwr > 0 ? leftPwr : 0);
	analogWrite(LEFT_MOTOR_BACK_PIN_NUM, rightPwr < 0 ? -rightPwr : 0);
	analogWrite(LEFT_MOTOR_FORTH_PIN_NUM, rightPwr > 0 ? rightPwr : 0);
}

// get distance on the ultrasonic sensor
float queryDist() {
	digitalWrite(ULTRASONIC_SENSOR_TRIG_PIN_NUM, HIGH);
	delayMicrosecond(10);
	digitalWrite(ULTRASONIC_SENSOR_TRIG_PIN_NUM, LOW);
	return pulseIn(ULTRASONIC_SENSOR_ECHO_PIN_NUM, HIGH) * 0.01715;
}

// wrap angle to [-pi, pi)
float wrapAngle(float angle) {
	angle += PI;
	while (angle < 0) {
		angle += 2 * PI;
	}
	while (angle >= 2 * PI) {
		angle -= 2 * PI;
	}
	return angle - PI;
}

// turn to an angle
void turn(float angle) {
	float lastAngle = Enes100.getTheta();
	while (true) {
		float curAngle = Enes100.getTheta();
		float diff = wrapAngle(angle - curAngle);
		if (abs(diff) <= ANGLE_TOLERANCE) {
			return;
		}
		int pwr = TURNING_BASE_PWR * (diff - TURNING_DAMPING_FACTOR * wrapAngle(curAngle - lastAngle));
		maneuver(-pwr, pwr);
		lastAngle = curAngle;
		delay(WAIT_TIME);
	}
}

// calculate distance to a point
float calcDist(float x, float y) {
	float curX = Enes100.getX();
	float curY = Enes100.getY();
	return sqrt((curX - x) * (curX - x) + (curY - y) * (curY - y));
}

// move somewhere; returns true if the movement succeeded, or false if an obstacle was detected
bool move(float x, float y) {
	float lastDist = calcDist(x, y);
	while (true) {
		float curDist = calcDist(x, y);
		if (curDist <= TARG_DIST_TOLERANCE) {
			return true;
		}
		turn(atan2(y - Enes100.getY(), x - Enes100.getX()))
		if (queryDist() <= OBSTACLE_DIST_TOLERANCE) {
			return false;
		}
		int pwr = MOVEMENT_BASE_PWR * (curDist - MOVEMENT_DAMPING_FACTOR * (curDist - lastDist));
		maneuver(pwr, pwr);
		lastDist = curDist;
		delay(WAIT_TIME);
	}
}

// do the mission
void doMission() {
}

// finish up
void completeMission() {
	// through the first column of obstacles
	move(1, Enes100.getY());
	move(Enes100.getX(), 1.5);
	if (!move(1.9, Enes100.getY())) {
		move(1, Enes100.getY());
		move(Enes100.getX(), 1);
		if (!move(1.9, Enes100.getY())) {
			move(1, Enes100.getY());
			move(Enes100.getX(), 0.5);
			move(1.9, Enes100.getY());
		}
	}
	// through the second column of obstacles
	move(Enes100.getX(), 1.5);
	if (!move(2.85, Enes100.getY())) {
		move(1.9, Enes100.getY());
		move(Enes100.getX(), 1);
		if (!move(2.85, Enes100.getY())) {
			move(1.9, Enes100.getY());
			move(Enes100.getX(), 0.5);
			move(2.85, Enes100.getY());
		}
	}
	// through the limbo
	move(Enes100.getX(), 1.5);
	move(3.7, Enes100.getY());
	maneuver(0, 0);
}

void setup() {
	/*Enes100.begin("Swiss Cheese", FIRE, ARUCO_MARKER_ID, 1116);
	pinMode(LEFT_MOTOR_BACK_PIN_NUM, OUTPUT);
	pinMode(LEFT_MOTOR_FORTH_PIN_NUM, OUTPUT);
	pinMode(RIGHT_MOTOR_BACK_PIN_NUM, OUTPUT);
	pinMode(RIGHT_MOTOR_FORTH_PIN_NUM, OUTPUT);
	pinMode(ULTRASONIC_SENSOR_TRIG_PIN_NUM, OUTPUT);
	pinMode(ULTRASONIC_SENSOR_ECHO_PIN_NUM, INPUT);
	doMission();
	completeMission();*/
	maneuver(100, 100);
}

void loop() {
	delay(1000);
}
