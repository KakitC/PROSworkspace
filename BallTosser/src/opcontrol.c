/** @file opcontrol.c
 * @brief File for operator control code
 *
 * This file should contain the user operatorControl() function and any functions related to it.
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

//Motor port definitions
#define LDRIVE 1 //Negative forwards
#define RDRIVE 10 //Positive forwards
#define TRIG 2 //Positive fire
#define LAUNCHA 4 //Positive fire
#define LAUNCHB 5 //Negative fire

//Sensors
#define TRIGPOT 1 //Analog

//RPi communication digital port definitions
#define READY 1
#define CMDA 2 //Least significant bit
#define CMDB 3
#define CMD0 4 //Least significant bit
#define CMD1 5
#define CMD2 6
#define CMD3 7

void turn(int, int);
void fire(void);

/*
 * Runs the user operator control code. This function will be started in its own task with the
 * default priority and stack size whenever the robot is enabled via the Field Management System
 * or the VEX Competition Switch in the operator control mode. If the robot is disabled or
 * communications is lost, the operator control task will be stopped by the kernel. Re-enabling
 * the robot will restart the task, not resume it from where it left off.
 *
 * If no VEX Competition Switch or Field Management system is plugged in, the VEX Cortex will
 * run the operator control task. Be warned that this will also occur if the VEX Cortex is
 * tethered directly to a computer via the USB A to A cable without any VEX Joystick attached.
 *
 * Code running in this task can take almost any action, as the VEX Joystick is available and
 * the schedular is operational. However, proper use of delay() or taskDelayUntil() is highly
 * recommended to give other tasks (including system tasks such as updating LCDs) time to run.
 *
 * This task should never exit; it should end with some kind of infinite loop, even if empty.
 */
void operatorControl() {
	bool isReady = 1;
	int command = 0;
	int dist = 0;

	//Prepare pins
	pinMode(READY, OUTPUT);
	pinMode(CMDA, INPUT);
	pinMode(CMDB, INPUT);
	pinMode(CMD0, INPUT);
	pinMode(CMD1, INPUT);
	pinMode(CMD2, INPUT);
	pinMode(CMD3, INPUT);

	while (1) {
		digitalWrite(READY, isReady);

		/* Commands
		 * 0: do nothing
		 * 1: Turn left
		 * 2: Turn right
		 * 3: Fire
		 */
		command = digitalRead(CMDA)
				  + digitalRead(CMDB)*2;
		if (command == 3) {
			isReady = 0;
			digitalWrite(READY, isReady);

			fire();

			isReady = 1;
		}
		else {
			dist = digitalRead(CMD0)
				   + digitalRead(CMD1) * 2
				   +  digitalRead(CMD2) * 4
				   +  digitalRead(CMD3) * 8;
			isReady = 0;
			digitalWrite(READY, isReady);

			turn(command - 1, dist);

			isReady = 1;
		}
	}
}

//dir = 0 -> left, dist is a delay time.
void turn(int dir, int dist) {
	float distScale = 32;
	int speed = 127;

	dist = dist*distScale;

	if (dir) {
		motorSet(LDRIVE, -speed);
		motorSet(RDRIVE, -speed);
		delay(dist);
		motorStop(LDRIVE);
		motorStop(RDRIVE);
	}
	else {
		motorSet(LDRIVE, speed);
		motorSet(RDRIVE, speed);
		delay(dist);
		motorStop(LDRIVE);
		motorStop(RDRIVE);
	}
}

void fire(void) {
	int triggerShoot = 300;
	int triggerReady = 500;

	//Spin up launcher wheel
	motorSet(LAUNCHA, 127);
	motorSet(LAUNCHB, -127);
	delay(500);

	while (analogRead(TRIGPOT) > triggerShoot) { //TODO Check which direction the pot goes in, and values
		motorSet(TRIG, 127);
	}
	while (analogRead(TRIGPOT) < triggerReady) {
		motorSet(TRIG, -127);
	}

	motorStop(TRIG);
	motorStop(LAUNCHA);
	motorStop(LAUNCHB);
}
