
//Sayraj Kazi
#pragma config(Sensor, S1,     touchSensor,    sensorEV3_Touch)
#pragma config(Sensor, S2,     gyroSensor,     sensorEV3_Gyro, modeEV3Gyro_RateAndAngle)
#pragma config(Sensor, S3,     colorSensor,    sensorEV3_Color, modeEV3Color_Color)
#pragma config(Sensor, S4,     sonarSensor,    sensorEV3_Ultrasonic)
#pragma config(Motor,  motorA,          armMotor,      tmotorEV3_Large, PIDControl, encoder)
#pragma config(Motor,  motorB,          leftMotor,     tmotorEV3_Large, PIDControl, driveLeft, encoder)
#pragma config(Motor,  motorC,          rightMotor,    tmotorEV3_Large, PIDControl, driveRight, encoder)


#define GoToGoal 1
#define AllStop 2
#define GoHome 3
#define AvoidObst 4

// -------------------------------------------------------------------//
// Lab 10 -  Go to Waypoints
// -------------------------------------------------------------------//
// On implementing the unicycle model for robot kinematics and updating the
// position and heading of the mobile robot.
// Global Variables

float distance = 0; // distance covered
int angle_gyro;           // current heading angle
float pos_x, pos_y; // variables for position in the x and y direction
float angle_odo;				// angle found from odometry
float angle;      // average measurement from the odo and gyro
int state =1; // latest state for the robot
string state_msg;

float wayPoints_x[] = {0};
float wayPoints_y[] = {3000};

bool avoid = false;

task Display()
{
	string msg;  // string to display the position and heading of the robot
	while(1)
	{
		sprintf(msg, "(%d, %d, %d)", pos_x, pos_y, angle);
	  displayTextLine(3, msg);
	  sprintf(msg, "state: %d", state);
	  displayTextLine(4, msg);
	  displayTextLine(5, state_msg);
	  sprintf(msg, "distane: %d", distance);
	  displayTextLine(6, msg);
	  sprintf(msg, " angle: %d, %d", angle_gyro);
	  displayTextLine(7, msg);
	  sprintf(msg, "angle odo: %d", angle_odo);
	  displayTextLine(8, msg);
		sleep(20);
	}
}

bool Driving(float t_dist)
{
	int kp_dist = 2;
	int kp_ang = 3;
	bool flag_dist;
	string msg;
	float motor_power = kp_dist * (t_dist - distance);
	float turn_ratio  = kp_ang  * (0);  // i.e. we keep the current angle
		if (abs(t_dist - distance)<.25)
		{ // update the flag to show the distance has been covered
			flag_dist = True;
			motor_power = 0;
	  	//writeDebugStreamLine("Reached the target distance.");
			sprintf(state_msg, "Reached %d in mm", t_dist);
	  	//displayTextLine(8, msg);

	  	return flag_dist;
		}

		else // if we are still not there or if we havent moved at all
		{
			flag_dist = False;
			sprintf(state_msg, "Still Driving", t_dist);

		}

setMotorSync(leftMotor, rightMotor, turn_ratio, motor_power);
//sleep(20);
return flag_dist;
}

void stopMoving()
{

	setMotorSync(leftMotor, rightMotor, 0, 0);
	sleep(100);
}

bool pointTurning(float t_angle)
{
	  int kp_angle = 4;
		int turn_ratio = 100;
		int motor_power = kp_angle * (t_angle - angle);
		bool flag_angle;



		if (abs(t_angle - angle)<= 2) //0.25
	  {
	  	// Update the flag to show the desired movement has completed
			flag_angle=True;
			sprintf(state_msg, "Rchd t angle %d", t_angle);

			return flag_angle;
		}
		else
		{
			flag_angle = False;
			sprintf(state_msg, "Tur to angle %d ", t_angle);

		}
setMotorSync(leftMotor, rightMotor, turn_ratio , motor_power);
return flag_angle;
}


task Sensing()
{// updates the sensor readings as well as
 // the position and heading estimate of the robot as well
	resetMotorEncoder(leftMotor);
	resetMotorEncoder(rightMotor);
	resetGyro(gyroSensor);
	float wheel_dist = 175.84; // distance in mm covered by wheel in one rotation
	float wheel_rad = 28;
	float whl2whl = 150; // mm
	float d_left_dist = 0;
	float	d_right_dist=0;
  float d_angle = 0;
	float dx, dy, d_distance;

	pos_x = 0;
	pos_y = 0;

	int left_deg =0; // variable for storing the degree of wheel turns in left wheel
	int	right_deg =0;// variable for storing the degree of wheel turns in right wheel


	int prev_angle = 0;

	distance = 0;  // resetting to zero every time a new movement is to be executed

	int t_sample_sensing =5;
	while(1)
	{
		if (getUSDistance(S4) < 10){
			avoid = true;
		}
		else{
			avoid = false;
		}
		// Heading Update for the robot

		angle_gyro = getGyroDegrees(gyroSensor);		// already providing the absolute angle value

		// Position Update for the robot

		// measuring the fresh reading modulus 360 every time the code is executed
		// to record the distance traveled by the robot in last sample time
		d_left_dist = (float) 2* pi *wheel_rad * (getMotorEncoder(leftMotor) - left_deg)/360; // in mm
		d_right_dist = (float) 2* pi * wheel_rad * (getMotorEncoder(rightMotor) - right_deg)/360; // in mm

		// instantaneous linear velocity
		d_distance = (float )(d_left_dist + d_right_dist) / 2; // in mm

		// instantaneous angular velocity
	  d_angle = (float) 57.13 * (d_left_dist - d_right_dist)/ (whl2whl);  // in degrees/sec.

		left_deg = getMotorEncoder(leftMotor);
		right_deg = getMotorEncoder(rightMotor);

		// if the angle is with respect to positive y-axis
		dx  =  d_distance * sinDegrees(angle);
		dy  =  d_distance * cosDegrees(angle);

		// integrating the dx and dy to get the updated position
		pos_x = pos_x + dx;
		pos_y = pos_y + dy;

	  // integration of linear and angular velocity to get distance covered and angle
		distance+=d_distance;
		angle_odo+=d_angle;
	  angle = (float) (angle_gyro * 0.80 + angle_odo * 0.2);

		sleep(t_sample_sensing);
	}
}

void GoAngle(float t_angle)
{
	bool flag_angle = False;
	while(flag_angle!=True)  // if you have not already turned to face the target point
		flag_angle = pointTurning(t_angle);
  sleep(100);
}

void GoDistance(float t_dist)
{
	bool flag_dist = 0;
	while (flag_dist!=True && avoid == false)
		flag_dist = Driving(t_dist);
}
task main()
{

	startTask(Display);
	startTask(Sensing);
	clearDebugStream();

	// initiatl state for the program
	state = 1;

	// initial values of the location
	pos_x = 0;
	pos_y = 0;
	angle =0;

	// initializing the target angle and distance

	float t_dist = 0;
	float t_angle = 0;
	float tar_x;
	float tar_y;

	// Ensuring no movement in the start

	bool flag_dist = 0;
	bool flag_angle = 0;
	bool flag_obs = 0;

	// WayPoints Tracker
	int wP_count = 0;
	int wP_size  = sizeof(wayPoints_x)/sizeof(wayPoints_x[0]);
	tar_x = wayPoints_x[wP_count];
	tar_y = wayPoints_y[wP_count];

	while(True)
	{// state update happens here
		sleep(30); // to let the sensing part catch up on the e_dist assignment

			switch(state)
			{

				case GoToGoal:   // First state: Turn to the Desired Heading

						// because of changed reference zero angle,
						t_angle =(float) atan2(tar_x - pos_x,tar_y - pos_y) * 57.13;


						GoAngle(t_angle);
						distance = 0;  // re-initializing the distance for calculations starting now
						t_dist = sqrt(pow(tar_x - pos_x,2)+ pow(tar_y - pos_y,2));
						t_angle = angle;   // keeping the current angle as the target for forward driving too
						GoDistance(t_dist);


						if (avoid == true){
									state = AvoidObst;
								}
					  else								// otherwise, go to AllStop condition
							state =GoHome;

						break;



				case AllStop:
						stopMoving();
						sprintf(state_msg, "Nothing to do");

						break;

				case GoHome:
						tar_x = 0;
						tar_y = 0;

						t_angle =(float) atan2(tar_x - pos_x,tar_y - pos_y) * 57.13;
						GoAngle(t_angle);
						distance = 0;  // re-initializing the distance for calculations starting now
						t_dist = sqrt(pow(tar_x - pos_x,2)+ pow(tar_y - pos_y,2));
						t_angle = angle;   // keeping the current angle as the target for forward driving too
						GoDistance(t_dist);

						// to orient back to the front
						GoAngle(0);
						state = AllStop;
					break;

				case AvoidObst:

				distance = 0;
					displayBigTextLine(4, "State: AvoidObst");
        	GoAngle(-90);
        	sleep(100);
        	GoDistance(1000);
        //	sleep(100);
        	GoAngle(9);
          sleep(100);
         // GoAngle(-90);
        	if (avoid == false){
        		state = GoToGoal;
        	}


        break;

			}

	}


}
