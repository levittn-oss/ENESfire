#include <Arduino.h>

// color sensor pin #s
#define COL_SENSOR_S0_PIN_NUM 0
#define COL_SENSOR_S1_PIN_NUM 0
#define COL_SENSOR_S2_PIN_NUM 0
#define COL_SENSOR_S3_PIN_NUM 0
#define COL_SENSOR_OUTPUT_PIN_NUM 0

// motor pin #s
#define LEFT_MOTOR_FORTH_PIN_NUM 0
#define LEFT_MOTOR_BACK_PIN_NUM 0
#define RIGHT_MOTOR_FORTH_PIN_NUM 0
#define RIGHT_MOTOR_BACK_PIN_NUM 0

// values to fine-tune
#define BLUE_LINE_DETECTION_THRESHOLD 16
#define MISSION_SITE_DETECTION_MOTOR_PWR 31
#define MISSION_SITE_DETECTION_THRESHOLD 0.2286

// ultrasonic sensor pin #s
#define DIST_SENSOR_TRIG_PIN_NUM 0
#define DIST_SENSOR_ECHO_PIN_NUM 0

// check the color sensor for the blue line
bool chkBlueLine() {

	// red = LOW/LOW
	digitalWrite(COL_SENSOR_S2_PIN_NUM, LOW);
	digitalWrite(COL_SENSOR_S3_PIN_NUM, LOW);
	unsigned long redTime = pulseIn(COL_SENSOR_OUTPUT_PIN_NUM, HIGH);

	// green = LOW/HIGH
	digitalWrite(COL_SENSOR_S2_PIN_NUM, LOW);
	digitalWrite(COL_SENSOR_S3_PIN_NUM, HIGH);
	unsigned long greenTime = pulseIn(COL_SENSOR_OUTPUT_PIN_NUM, HIGH);

	// blue = HIGH/HIGH
	digitalWrite(COL_SENSOR_S2_PIN_NUM, HIGH);
	digitalWrite(COL_SENSOR_S3_PIN_NUM, HIGH);
	unsigned long blueTime = pulseIn(COL_SENSOR_OUTPUT_PIN_NUM, HIGH);

	// comparison
	return blueTime - redTime >= BLUE_LINE_DETECTION_THRESHOLD && blueTime - greenTime >= BLUE_LINE_DETECTION_THRESHOLD;
}

// get distance on the ultrasonic sensor
float queryDist() {

	// let the sensor reset
	delayMicrosecond(2);

	// activate
	digitalWrite(DIST_SENSOR_TRIG_PIN_NUM, HIGH);
	delayMicrosecond(10);
	digitalWrite(DIST_SENSOR_TRIG_PIN_NUM, LOW);

	// time -> distance
	return pulseIn(ULTRASONIC_SENSOR_ECHO_PIN_NUM, HIGH) * 0.01715;
}

// follow the blue line to the mission site and prepare to take pictures
void findMissionSite() {

	// pin setup
	pinMode(COL_SENSOR_S0_PIN_NUM, OUTPUT);
	pinMode(COL_SENSOR_S1_PIN_NUM, OUTPUT);
	pinMode(COL_SENSOR_S2_PIN_NUM, OUTPUT);
	pinMode(COL_SENSOR_S3_PIN_NUM, OUTPUT);
	pinMode(COL_SENSOR_OUTPUT_PIN_NUM, INPUT);
	pinMode(LEFT_MOTOR_FORTH_PIN_NUM, OUTPUT);
	pinMode(LEFT_MOTOR_BACK_PIN_NUM, OUTPUT);
	pinMode(RIGHT_MOTOR_FORTH_PIN_NUM, OUTPUT);
	pinMode(RIGHT_MOTOR_BACK_PIN_NUM, OUTPUT);
	pinMode(DIST_SENSOR_TRIG_PIN_NUM, OUTPUT);
	pinMode(DIST_SENSOR_ECHO_PIN_NUM, INPUT);

	// sensor setup
	digitalWrite(COL_SENSOR_S0_PIN_NUM, HIGH);
	digitalWrite(COL_SENSOR_S1_PIN_NUM, HIGH);
	digitalWrite(DIST_SENSOR_TRIG_PIN_NUM, LOW);

	// turn
	analogWrite(LEFT_MOTOR_FORTH_PIN_NUM, MISSION_SITE_DETECTION_MOTOR_PWR);
	analogWrite(LEFT_MOTOR_BACK_PIN_NUM, 0);
	analogWrite(RIGHT_MOTOR_FORTH_PIN_NUM, 0);
	analogWrite(RIGHT_MOTOR_BACK_PIN_NUM, MISSION_SITE_DETECTION_MOTOR_PWR);

	while (true) {

		// stop turning and go forward when required
		while (!chkBlueLine()) {
		}

		// left motor is already going forward
		analogWrite(RIGHT_MOTOR_FORTH_PIN_NUM, MISSION_SITE_DETECTION_MOTOR_PWR);
		analogWrite(RIGHT_MOTOR_BACK_PIN_NUM, 0);

		// turn again if required, or end if the mission site is here
		while (chkBlueLine()) {
			if (getUltrasonicDist() <= MISSION_SITE_DETECTION_THRESHOLD) {
				return;
			}
		}

		// left motor is already going forward
		analogWrite(RIGHT_MOTOR_FORTH_PIN_NUM, 0);
		analogWrite(RIGHT_MOTOR_BACK_PIN_NUM, MISSION_SITE_DETECTION_MOTOR_PWR);

	}
}
