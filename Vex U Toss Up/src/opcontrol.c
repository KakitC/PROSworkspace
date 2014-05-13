/* edit by wphan
* added push button limits
* trimpot to keep arm up during idle
*/
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
#include "api.h"

#define DRIVE_FL 6 //Left drive 127 is forward
#define DRIVE_ML 7
#define DRIVE_MR 8 //right drive -127 is forward
#define DRIVE_FR 9

#define ARM_TL 2 //-127 is up? lolsure
#define ARM_TR 3 //-127 is up
#define ARM_BL 4 //-127 is up
#define ARM_BR 5 //127 is up
#define arm_idle_speed 8

#define IN_L 1 //127 should be intake
#define IN_R 10

#define led_r 6
#define led_g 8
#define limit_top 3
#define limit_bot 4
#define arm_pot 1
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
	int current_pot_val=analogRead(arm_pot);

	if (digitalRead(2)==LOW){ //JUMPER IN DIG2=GO AUTONOMOUs
		autonomous();
		digitalWrite(limit_bot, HIGH);
		digitalWrite(limit_top, HIGH);
	}

	while (1) {
		//Drive motors, tank config
		motorSet(DRIVE_FL, joystickGetAnalog(1,3));
		motorSet(DRIVE_ML, joystickGetAnalog(1,3));
		motorSet(DRIVE_FR, -joystickGetAnalog(1,2));
		motorSet(DRIVE_MR, -joystickGetAnalog(1,2));


		//Arm motors, right trigger buttons
		if((joystickGetDigital(1,6,JOY_UP) == 1) && (digitalRead(limit_top) == 1)) {
			current_pot_val=analogRead(arm_pot);
			motorSet(ARM_TL, -127);
			motorSet(ARM_BL, -127);
			motorSet(ARM_TR, -127);
			motorSet(ARM_BR, 127);
			if(digitalRead(limit_top)==0) {
				motorStop(ARM_TL);
				motorStop(ARM_BL);
				motorStop(ARM_TR);
				motorStop(ARM_BR);
			}
		}
		else if((joystickGetDigital(1,6,JOY_DOWN) == 1) && (digitalRead(limit_bot) == 1))  {
			current_pot_val=analogRead(arm_pot);
			motorSet(ARM_TL, 127);
			motorSet(ARM_BL, 127);
			motorSet(ARM_TR, 127);
			motorSet(ARM_BR, -127);
			if(digitalRead(limit_bot)==0) {
				motorStop(ARM_TL);
				motorStop(ARM_BL);
				motorStop(ARM_TR);
				motorStop(ARM_BR);
			}
		}
		else {//keep arm up
			if(analogRead(arm_pot)<=current_pot_val){
			//higher than idle
				motorSet(ARM_TL, arm_idle_speed);
				motorSet(ARM_BL, arm_idle_speed);
				motorSet(ARM_TR, arm_idle_speed);
				motorSet(ARM_BR, -(arm_idle_speed));
				if((analogRead(arm_pot)==current_pot_val)||(digitalRead(limit_bot)==0)){
					motorStop(ARM_TL);
					motorStop(ARM_BL);
					motorStop(ARM_TR);
					motorStop(ARM_BR);
				}
			}
			if(analogRead(arm_pot)>=current_pot_val){
				//lower than idle
				motorSet(ARM_TL, -(arm_idle_speed));
				motorSet(ARM_BL, -(arm_idle_speed));
				motorSet(ARM_TR, -(arm_idle_speed));
				motorSet(ARM_BR, (arm_idle_speed));
				if((analogRead(arm_pot)==current_pot_val)||(digitalRead(limit_top)==0)){
					motorStop(ARM_TL);
					motorStop(ARM_BL);
					motorStop(ARM_TR);
					motorStop(ARM_BR);
				}
			}
		}

		//Intake motors, left trigger buttons
		if(joystickGetDigital(1,5,JOY_UP) == 1) {
			motorSet(IN_L, 127);
			motorSet(IN_R, -127);
		}
		else if(joystickGetDigital(1,5,JOY_DOWN) == 1) {
			motorSet(IN_L, -127);
			motorSet(IN_R, 127);
		}
		else {
			motorStop(IN_L);
			motorStop(IN_R);
		}


		//preset arm heights
		if(joystickGetDigital(1, 7,JOY_UP) == 1) {
			while((analogRead(arm_pot)>2202) && (digitalRead(limit_top) == 1)){
				current_pot_val=2202;
				motorSet(ARM_TL, -127);
				motorSet(ARM_BL, -127);
				motorSet(ARM_TR, -127);
				motorSet(ARM_BR, 127);
				if(digitalRead(limit_top)==0) {
					motorStop(ARM_TL);
					motorStop(ARM_BL);
					motorStop(ARM_TR);
					motorStop(ARM_BR);
				}
			}
		}
		if(joystickGetDigital(1, 7,JOY_DOWN) == 1) {
			while((analogRead(arm_pot)<4040) && (digitalRead(limit_bot) == 1)){
				current_pot_val=4040;
				motorSet(ARM_TL, 127);
				motorSet(ARM_BL, 127);
				motorSet(ARM_TR, 127);
				motorSet(ARM_BR, -127);
				if(digitalRead(limit_bot)==0) {
					motorStop(ARM_TL);
					motorStop(ARM_BL);
					motorStop(ARM_TR);
					motorStop(ARM_BR);
				}
			}
		}
		//LIGHTS
		if(digitalRead(limit_top)==0)
			digitalWrite(led_r, LOW);
		else
			digitalWrite(led_r, HIGH);

		if(digitalRead(limit_bot)==0)
			digitalWrite(led_g, LOW);
		else
			digitalWrite(led_g, HIGH);

	}


		printf("%d\r\n", current_pot_val);
	}

