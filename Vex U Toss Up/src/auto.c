/** @file auto.c
* @brief File for autonomous code
*
* This file should contain the user autonomous() function and any functions related to it.
*
* Copyright (c) 2011-2013, Purdue University ACM SIG BOTS.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*     * Neither the name of Purdue University ACM SIG BOTS nor the
*       names of its contributors may be used to endorse or promote products
*       derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL PURDUE UNIVERSITY ACM SIG BOTS BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* Purdue Robotics OS contains FreeRTOS (http://www.freertos.org) whose source code may be
* obtained from http://sourceforge.net/projects/freertos/files/ or on request.
*/

#include "main.h"
#include "api.h"

#define DRIVE_FL 6 //Left drive 127 is forward
#define DRIVE_ML 7
#define DRIVE_MR 8 //right drive -127 is forward
#define DRIVE_FR 9

#define ARM_TL 2 //-127 is up? lolsure
#define ARM_TR 3 //-127 is up
#define ARM_BL 4 //-127 is up
#define ARM_BR 5 //127 is up
#define ARM_IDLE_SPEED 8

#define ARM_POS_BOT 4000
#define ARM_POS_LOW 3550
#define ARM_POS_MID 3100
#define ARM_POS_TOP 2000

#define IN_L 1 //-127 intake
#define IN_R 10

#define LIMIT_TOP 3
#define LIMIT_BOT 2
#define COLOUR_JUMPER 9

#define ARM_POT 1
#define LINESENSE_L 2
#define LINESENSE_R 3

#define LINE_THRESH 300

#define IME_LEFT 0
#define IME_RIGHT 1

#define LED_R 6
#define LED_g 8

Ultrasonic ultraFront;


void motorsLeft(int speed);
void motorsRight(int speed);
void motorsArm(int speed);
void intake(void);
void outtake(void);
void stopDrive(void);
void stopArm(void);
void stopIntake(void);
void clearEncoders (void);
void driveStraight(int dist, int speed);
void driveToLine(bool forwards);
void driveTurn(int angle, int speed, bool colour);
void driveTurn90(bool dir, bool colour);
void driveBrake(void);
void driveDeadReckon(int speedL, int speedR, int time);
void armTo(int pos, int speed);
void stopEmergency(void);
void driveStop(void);


/*
* Runs the user autonomous code. This function will be started in its own task with the default
* priority and stack size whenever the robot is enabled via the Field Management System or the
* VEX Competition Switch in the autonomous mode. If the robot is disabled or communications is
* lost, the autonomous task will be stopped by the kernel. Re-enabling the robot will restart
* the task, not re-start it from where it left off.
*
* Code running in the autonomous task cannot access information from the VEX Joystick. However,
* the autonomous function can be invoked from another task if a VEX Competition Switch is not
* available, and it can access joystick information if called in this way.
*
* The autonomous task may exit, unlike operatorControl() which should never exit. If it does
* so, the robot will await a switch to another mode or disable/enable cycle.
*/

/*
************AUTONOMOUS ROUTINE:*******************
*1. Ram backwards into large balls, pushing into opponents
*2. Drive angled towards bump and push small balls over
*3. Pick up preload and turn to pick up 2 small balls under bar
*4. Drive back over bump, turn to face goals and align
*5. Drive to goal, using lines
*6. Align with goal, confirm positioning and score 3 balls
*7. Turn around, and swing intake about, hopefully hitting large balls over bridge
*/
void autonomous() {
	bool colour = digitalRead(COLOUR_JUMPER); //COLOUR_JUMPER 9
	//BLUE IS 0, RED IS 1. code as for BLUE;;0 == jumperIN, 1 == jumperOUT
	long currentTime;
	long startTime;

	clearEncoders();


	if (digitalRead(5)==LOW){//prints things to screen in absence of auton

		while(1) {
			int armPos, count, countL, countR, lineL, lineR, ultraDist;
			lineL = analogRead(LINESENSE_L);
			lineR = analogRead(LINESENSE_R);
			imeGet(0, &count);
			imeGet(IME_LEFT, &countL);
			imeGet(IME_RIGHT, &countR);
			armPos = analogRead(ARM_POT);
			ultraDist = ultrasonicGet(ultraFront);

			printf(/*"Arm:%d, IME_L:%d, IME_R:%d, */"UltraDist:%d, LineL:%d, LineR:%d\r\n",
					/*armPos, countL, countR,*/ ultraDist, lineL, lineR);
			delay(100);
		}
	}


	armTo(ARM_POS_LOW, 127);
	armTo(ARM_POS_BOT, 60);


	//RAMMING AUTON, NO PICK UP 2 ON BACK WALL
	if (!digitalRead(12)) { //RAM JUMPER 12 IN
		//Raise arm to release intake rollers


		//RAMMING SPEED
		driveDeadReckon(-127,-127,1900);
		intake();

		driveBrake();
		delay(500); //extra delay?
		driveTurn(10, 50, colour);
	}



	//MAIN SCORING AUTON ROUTINE
	else {
		//Intake 2 balls off wall
		intake();
		driveStraight(480,45);
		driveStop();
		delay(1000);
		driveStraight(-50,20);

		if (colour) { //Turn to ram
			driveDeadReckon(20, -60, 700);
		} else {
			driveDeadReckon(-60, 20, 700);
		}
		delay(500);
		//driveTurn(10,60,colour);


		//RAMMING SPEED! Dead reckon to middle, come back and turn slightly into bump
		driveDeadReckon(-127,-127,1700);

		driveBrake();
		delay(500); //extra delay?
		driveTurn(15, 50, colour);
	}


	startTime = millis(); //Line based go back with timeout
	//Colour based, uses linesensor that won't cross over horizontal line
	//Should really drive away from bump, then align on the first vertical line away from the bump
	int sense;
	if (colour) {
		sense = analogRead(LINESENSE_L);
	} else {
		sense = analogRead(LINESENSE_R);
	}
	do {
		currentTime = millis();
		driveDeadReckon(50,50,1);
		if (colour) {
				sense = analogRead(LINESENSE_L);
			} else {
				sense = analogRead(LINESENSE_R);
			}
	} while ( (sense > LINE_THRESH) && (currentTime - startTime < 3500));
	if (!(currentTime - startTime < 3500)) {
		stopEmergency();
	}

	driveStraight(80,40); //Get off the last line
	driveToLine(1); //TODO add timeout //Hit the tile
	driveStop();

	//Turn to face bump/goal
	armTo(ARM_POS_LOW, 127);
	driveStraight(-200, 30);
	if(!colour)
		driveTurn(60, 127, 0);
	else
		driveTurn(135, 127, 1);
	driveBrake();

	//Align to bump in front
	driveDeadReckon(30,30,1000);

	//Go over bump, realign to it
	//driveStraight(-100,50); //Back up for RAMMING SPEED
	driveDeadReckon(127,127,1500);
	driveDeadReckon(-30,-30,2000);
	driveStop();
	armTo(ARM_POS_BOT, 60);
	delay(300);

#if 0
	if (colour) {
		driveDeadReckon(-30,40,150);
	} else {
		driveDeadReckon(40,-30,150);
	}

	driveStraight(200, 50);
	stopDrive();

	//Drive face into wall, back up, turn to face back, and back up to align with bump
	stopIntake(); //Maneuvers don't account for having to pick preload back up if we don't carry it to start
	armTo(ARM_POS_TOP, 127);
	driveDeadReckon(30,30,2000);
	driveStraight(-200,30);
	driveTurn(-90, 50, colour);



	driveDeadReckon(-20, -20, 800);
	stopDrive();

	//Intake 2 balls on back wall
	armTo(ARM_POS_BOT, 127);
	intake();
	driveStraight(800, 80);
	driveStraight(600, 30);
	driveBrake();
	delay(500);
	stopIntake();

	//Drive backwards over bump, drive forwards to align with bump
	driveStraight(-1000, 127);
	stopDrive();
	armTo(ARM_POS_LOW, 127); //Kind of up to go over bump
	driveDeadReckon(-127,-127,1500);
	driveBrake();
	driveDeadReckon(20, 20, 800);

	//Drive under bridge to line
	driveStraight(-3000, 127);
	driveToLine(0);

	//Turn, go to line in front of goal
	driveTurn90(0, colour);
	driveStraight(500,127);
	driveToLine(1);
	driveTurn90(1, colour);
#endif

	//Drive under bridge to line, then some more
	startTime = millis(); //Line based go to bridge with timeout
	//Colour based, uses linesensor that won't cross over vertical line
	//Doesn't need to be unless we bias it a lot
	currentTime = millis();
	if (colour) {
		sense = analogRead(LINESENSE_R);
	} else {
		sense = analogRead(LINESENSE_L);
	}
	do {
		currentTime = millis();
		driveDeadReckon(80,80,1);
		if (colour) {
				sense = analogRead(LINESENSE_R);
			} else {
				sense = analogRead(LINESENSE_L);
			}
	} while ( (sense > LINE_THRESH || sense < 50) && (currentTime - startTime < 2000));
	if (!(currentTime - startTime < 2000)) {
		stopEmergency();
	}

	driveStraight(500, 50); //drive out from under bridge

	//Drive up to goal, make sure goal is there with timeout and score
	armTo(1000,127); //All the way up
	int ultraDistance = ultrasonicGet(ultraFront);
	currentTime = millis();
	startTime = millis();
	while ((!((ultraDistance < 16) && (ultraDistance > 10))) && ((currentTime - startTime) < 4000)) {
		driveDeadReckon(30,30,1);
		currentTime = millis();
		ultraDistance = ultrasonicGet(ultraFront);
	}
	if ((currentTime - startTime) < 4000) {
		driveStop();
		motorSet(IN_L, 127);
		motorSet(IN_R, -127);
		//outtake();
		delay(5000);
	} else {
		stopEmergency();
	}


	//Knock large balls off the bridge
	driveStraight(-250, 50);
	driveTurn(270, 50, !colour);
	driveStraight(250, 60);
	driveBrake();
	armTo(ARM_POS_MID, 50);
	driveTurn(150, 70, colour);
	driveTurn(-300, 70, colour);


	stopEmergency(); //END

} //End of autonomous()



/////functions///////////////////////////////////////////////////////////////////////////////////////
void motorsLeft(int speed){
	motorSet(DRIVE_FL, speed);
	motorSet(DRIVE_ML, speed);
	return;
}
void motorsRight(int speed){
	motorSet(DRIVE_FR, -speed);
	motorSet(DRIVE_MR, -speed);
	return;
}

//positive is up
void motorsArm(int speed){
	motorSet(ARM_TL, -speed);
	motorSet(ARM_TR, -speed);
	motorSet(ARM_BL, -speed);
	motorSet(ARM_BR, speed);
	return;
}

void intake(void){
	motorSet(IN_L, -127);
	motorSet(IN_R, 127);
	return;
}

void outtake(void){
	motorSet(IN_L, 127);
	motorSet(IN_R, -127);
	return;
}

void stopDrive(void){
	motorStop(DRIVE_FL);
	motorStop(DRIVE_ML);
	motorStop(DRIVE_FR);
	motorStop(DRIVE_MR);
	return;
}

void driveStop(void){
	stopDrive();
	return;
}

void stopArm(void){
	motorSet(ARM_TL, -ARM_IDLE_SPEED);
	motorSet(ARM_TR, -ARM_IDLE_SPEED);
	motorSet(ARM_BL, -ARM_IDLE_SPEED);
	motorSet(ARM_BR, ARM_IDLE_SPEED);
	return;
}
void stopIntake(void){
	motorStop(IN_L);
	motorStop(IN_R);
	return;
}

void stopEmergency(void){
	stopArm();
	stopDrive();
	stopIntake();
	delay(100000);
}

void clearEncoders (void){
	imeReset(0);
	imeReset(1);
	return;
}


//Drives a set distance with straightness correction
//dist: distance to travel, 620 = 1 revolution, positive is forwards, negative back
//speed: valid range 0 to 127, but don’t use small values or it won’t move
void driveStraight(int dist, int speed) {
	int countL;
	int countR;
	int speedAdj;

	if (dist < 0) {
		speed = -speed;
	}
	clearEncoders();
	do {
		imeGet(IME_LEFT, &countL);
		countL = -countL; //left encoder is reversed
		imeGet(IME_RIGHT, &countR);
		speedAdj = (countL - countR) * 1; //Speed adjustment factor

		motorsLeft(speed - speedAdj);
		motorsRight(speed + speedAdj);

	} while ( (dist < 0 && (countL+countR)/2 > dist) || (dist > 0 && (countL+countR)/2 < dist));

	stopDrive();
	return;
}

//Drives until a front corner line sensor hits a line. Then turns so that both are on the line
//forwards: True drives robot forwards to line, false drives back
void driveToLine(bool forwards) {
	int senseR, senseL;

	int speedBack = -15;
	int speed = 25;

	if (!forwards) {
		speed = -speed;
		speedBack = -speedBack;
	}

	senseR = analogRead(LINESENSE_R);
	senseL = analogRead(LINESENSE_L);
	while ( (senseR > LINE_THRESH) && (senseL > LINE_THRESH) ) {
		senseR = analogRead(LINESENSE_R);
		senseL = analogRead(LINESENSE_L);
		motorsRight(speed);
		motorsLeft(speed);
	}

	if ( senseR <= LINE_THRESH && senseL > LINE_THRESH ) { //TODO Add feedback loop here, not recursive correction below
		while ( senseL > LINE_THRESH && senseR <= LINE_THRESH ) { //Should correct overshoot
			senseR = analogRead(LINESENSE_R);			//Also, needs to bias towards direction it came from to not get lost
			senseL = analogRead(LINESENSE_L);
			motorsRight(speedBack); //TODO Add timeout for feedback loop
			motorsLeft(speed);
		}
	}
	else if ( senseR > LINE_THRESH && senseL <= LINE_THRESH ){
		while (senseR > LINE_THRESH && senseL <= LINE_THRESH ) {
			senseR = analogRead(LINESENSE_R);
			senseL = analogRead(LINESENSE_L);
			motorsLeft(speedBack);
			motorsRight(speed);
		}
	}
	driveBrake();

	/*senseR = analogRead(LINESENSE_R);
	senseL = analogRead(LINESENSE_L);
	if(senseL < LINE_THRESH && senseR < LINE_THRESH)
		return;
	else if(forwards==1){ //RECURSIVE CORRECTION (kind of)
		driveToLine(0); //drive backwards to correct overshoot
		return;}
	else if(forwards==0){
		driveToLine(1); //drive fowards to correct overshoot
		return;}*/
	return;
}



//Turns approximately the correct angle in a given direction on the spot. Not precise, no angle correction
//Angle: positive is left (CCW), negative is right (CW), Approximately in degrees
//Speed: from 0-127
//Colour: Assume Blue when coding for colour=LOW. Red is HIGH
void driveTurn(int angle, int speed, bool colour){
	int dist;
	double degToEncoder = 2.5;

	if ( colour == HIGH ){
		dist = -angle*degToEncoder;
	}
	else{
		dist = angle*degToEncoder; //left wheel drives back, negative encoder count
	}


	int count; //uses one encoder only
	clearEncoders();
	if (dist > 0) {
		do{
			imeGet(IME_RIGHT, &count);
			motorsRight(speed);
			motorsLeft(-speed);
		}while (count < dist);
	}
	else {
		do{
			imeGet(IME_LEFT, &count);
			motorsRight(-speed);
			motorsLeft(speed);
		}while (count > dist);
	}
	return;
}


//DOESN'T REALLY WORK
//Precise 90 degree turning function
//dir: 0 is CCW (left), 1 is CW (right)
void driveTurn90(bool dir, bool colour){
	int dist = 150; //90 Degree precise turn value
	int speed = 50; //90 degree max precise turning speed
	int countL;
	int countR;

	if ( colour ) {
		if (dir == 1) {
			dir = 0;
		} else {
			dir = 1;
		}
	}

	//Line up pivot point over line
	//driveStraight(440,speed);
	driveBrake();
	delay(250);

	clearEncoders();
	if ( dir == 0 ){ //CCW left
		do{
			imeGet(IME_LEFT, &countL);
			countL = -countL;
			motorsLeft(-speed);
			motorsRight(speed);
			printf("dist%d", countL);
		}while (countL < dist);
		driveBrake();
	}
	else{ //CW Right
		do{
			imeGet(IME_RIGHT, &countR);
			motorsLeft(speed);
			motorsRight(-speed);
			printf("dist%d", countR);
		}while (countR < dist);
		driveBrake();
	}
	//TODO: Implement left-right distance correction (one turning too fast) after fixing dual IMEs
	return;
}


//Gets the last velocity of the wheels and applies braking speeds for a short time, then stops
void driveBrake(void){
	int velL;
	int velR;
	double brakeConst = .24;
	bool encoderL = imeGetVelocity(IME_LEFT, &velL);
	bool encoderR = imeGetVelocity(IME_RIGHT, &velR);

	if (encoderL) { //Included a thing to make sure that it gets the IME velocity, or else just stops dumbly
		motorsLeft(-velL*brakeConst);
	} else { motorsLeft(0);}

	if (encoderR) {
		motorsLeft(-velR*brakeConst);
	} else { motorsRight(0);}

	delay(150);
	stopDrive();

	return;
}

//LOLOLOL GO FAST
//Time dependent driving. For imprecise maneuvers
void driveDeadReckon(int speedL, int speedR, int time) {
	motorsLeft(speedL);
	motorsRight(speedR);
	delay(time);
}

//Moves arm to given position, at speed, up to limit switches
//Valid ranges for speed: ~10-127, don't put small numbers or it won't move
//TOPpos: 2300-2315
//MIDpos: 3110-3133
//BOTpos: 3990-4020

void armTo(int pos, int speed) {
	int currentPos = analogRead(ARM_POT);
	int posThresh = 10; //Within posThresh of actual value

	if (currentPos < pos) {
		while (((currentPos + posThresh) < pos/* && digitalRead(LIMIT_BOT)==1*/)) {
			currentPos = analogRead(ARM_POT); //TODO Test bottom limit switch integrity
			motorsArm(-speed);
		}
	}
	else if(currentPos > pos) {
		while (((currentPos - posThresh) > pos) && digitalRead(LIMIT_TOP)==1) {
			currentPos = analogRead(ARM_POT);
			motorsArm(speed);
		}
	}

	stopArm();
return;
}



