/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.SquireBot;


import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="teleop from Autonomous", group="Class")
//@Disabled
public class servo_Tele_class extends LinearOpMode {

    // Declare variables used by the class
    Squirebot myBot;
    private double headingError  = 0;
    SparkFunOTOS.Pose2D pos;

    @Override public void runOpMode() {
        myBot = new Squirebot(hardwareMap,telemetry);
        myBot.drive.maxSpeed =0.7;
        myBot.GPS.UpdateGPS();
        pos = myBot.GPS.GPS;
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.addData("X coordinate", pos.x);
        telemetry.addData("Y coordinate", pos.y);
        telemetry.addData("Heading angle", pos.h);
        telemetry.update();

        double rotatorTarget=.5;

        //Start of TeleOp
        waitForStart();
        myBot.startTele();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            myBot.GPS.UpdateGPS();
            pos = myBot.GPS.GPS;
            myBot.arm.updateEncoders();//avoid repeated calls to read encoders to reduce loop time

            if (gamepad1.back) {
                resetCompass();
            }

            if (gamepad2.left_stick_y < -.15) {
                if (myBot.arm.armLiftLocation < 1800)
                    armLiftTarget = armLiftTarget - Math.round(gamepad2.left_stick_y*12);
            }
            if (gamepad2.left_stick_y > .15) {
                if (myBot.arm.armLiftLocation > 0) {
                    if (armExtendLocation > 1900) {
                      if (myBot.arm.armLiftLocation > 1200) {
                          armLiftTarget = armLiftTarget - Math.round(gamepad2.left_stick_y * 12);

                      }
                    }
                 else
                    armLiftTarget = armLiftTarget - Math.round(gamepad2.left_stick_y * 12);
                }
            }

            if (gamepad2.dpad_up)
                myBot.arm.setArmLiftTarget(1700);
            else if (gamepad2.dpad_right)
            {
                if (myBot.arm.armExtendLocation < 1000)
                {
                    myBot.arm.setArmLiftTarget(120);
                    rotatorTarget = 0.41;
                }
            }
            else if (gamepad2.dpad_down)
            {
                if (myBot.arm.armExtendLocation < 1900)
                    myBot.arm.setArmLiftTarget(0);
            }
            else if (gamepad2.dpad_left)
            {
                rotatorTarget= 0.56;
                myBot.arm.setArmLiftTarget(1174);

            } else if (gamepad2.right_trigger > 0.5) {
                rotatorTarget=0.33;
                myBot.arm.setArmLiftTarget(775);

            }
            //ARM EXTEND - This code prevents us from extending the arm too far when horizontal



            //rotator location
            if (gamepad2.x)
                rotatorTarget =0.73;
            else if (gamepad2.y)
                rotatorTarget =0.5;

            if (gamepad1.left_bumper)
                myBot.drive.maxSpeed = 0.2;
            else if (gamepad1.right_bumper) {
                myBot.drive.maxSpeed = 1;
            } else {
                myBot.drive.maxSpeed = 0.7;
            }
            myBot.drive.driveFC(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x,myBot.GPS.GPS.h);

                myBot.claw.setWristPosition(rotatorTarget);
                if (gamepad1.left_trigger > 0.5){
                    myBot.claw.closeGrabber();
                } else if (gamepad1.right_trigger > 0.5) {
                    myBot.claw.openGrabber();
                }

                telemetry.addData("Motor Encoders lift:extend", "%7d :%7d",
                        myBot.arm.armLiftLocation,
                        myBot.arm.armExtendLocation);
                telemetry.update();
            }
        }
        private void resetCompass(){
            telemetry.addData("Reseting IMU", "wait");
            telemetry.update();
            myBot.drive.drive(0,0,0);
            sleep(150);
            myBot.GPS.resetGPS();
        }



}
