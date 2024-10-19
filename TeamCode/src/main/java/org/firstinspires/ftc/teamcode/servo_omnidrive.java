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

package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//hoi
//Henry is a bot
//Ephan Is A Stinky Robotman
//henry is a gamer during robotics
//henry is still a bot
//bekett is slow in cross country
//Efan and Henwy are da best
//codes name is Servo Omni

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Servo Omni", group="Linear OpMode")
//@Disabled
public class servo_omnidrive extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private Servo grabber = null;
    private boolean grab_mode = false;
    NormalizedColorSensor colorSensor;
    private DcMotor armLift = null;
    private DcMotor armExtend = null;
    View relativeLayout;
    private double armLiftPower;
    private double armExtendPower;
    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        //left_drive  = hardwareMap.get(DcMotor.class, "left_drive");
        //right_drive = hardwareMap.get(DcMotor.class, "right_drive");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        grabber = hardwareMap.get(Servo.class,"grabber");
        armLift = hardwareMap.get(DcMotor.class, "arm_lift");
        armExtend = hardwareMap.get(DcMotor.class, "arm_extend");
        float gain = 2;
        final float[] hsvValues = new float[3];
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");


        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        armLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armExtend.setDirection(DcMotor.Direction.REVERSE);
        armLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        int armLiftLocation;
        int armExtendLocation;
        grabber.setPosition(0.3);

        waitForStart();
        colorSensor.setGain(gain);
        runtime.reset();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //ARM LIFT
            armLiftLocation=armLift.getCurrentPosition();
            armExtendLocation = armExtend.getCurrentPosition()
            double armLiftSpeed = gamepad2.left_stick_y*-0.5;
            armLift.setPower(armLiftSpeed);
            if (armLiftLocation < 200)
            {
                armLiftPower = gamepad2.left_stick_y*0.4 + 0.6;
            } else if (armLiftLocation <400) {
                armLiftPower = gamepad2.left_stick_y*0.4 + 0.4;
            }
            else if (armLiftLocation < 600) {
                armLiftPower = gamepad2.left_stick_y*0.4 + 0.2;
            }
            else
            {
                armLiftPower = gamepad2.left_stick_y*0.15;
            }
            */
            //ARM EXTEND
            double armExtendSpeed = gamepad2.right_stick_y*0.6;
            armExtend.setPower(armExtendSpeed);

            if(armLiftLocation < 300)
            {
                if (armExtendLocation< 100)
                {
                    if (gamepad2.a)
                        armExtendPower=1;
                    else(gampad2.b)
                            armExtendPower=0;
                }
                else if(armExtendLocation < 3500)
                {
                    if (gamepad2.a)
                        armExtendPower=1;
                    else(gampad2.b)
                            armExtendPower=-1;
                }
                else
                {
                    if (gamepad2.a)
                        armExtendPower=0;
                    else(gampad2.b)
                            armExtendPower=-1;
                }
            } else if (armLiftLocation > 300) {
                if (armExtendLocation< 100)
                {
                    if (gamepad2.a)
                        armExtendPower=1;
                    else(gampad2.b)
                            armExtendPower=0;
                }
                else if(armExtendLocation < 6500)
                {
                    if (gamepad2.a)
                        armExtendPower=1;
                    else(gampad2.b)
                            armExtendPower=-1;
                }

                else if(armExtendLocation < 8500)
                {
                    if (gamepad2.a)
                        armExtendPower=0.5;
                    else(gampad2.b)
                            armExtendPower=-0.5;
                else
                {
                    if (gamepad2.a)
                        armExtendPower=0;
                    else(gampad2.b)
                            armExtendPower=-1;
                }
            }


            double max;
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = -gamepad1.left_stick_x;
            double yaw = -gamepad1.right_stick_x;
            if(gamepad2.a)
            {
                grab_mode = true;
            }
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }



            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
            armExtend.setPower(armExtendPower);
            armLift.setPower(armLiftPower);
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            Color.colorToHSV(colors.toColor(), hsvValues);
            //what if we simplified this so it only said RED, BLUE, or YELLOW? HI
            telemetry.addLine()
                    .addData("Red", "%.3f", colors.red)
                    .addData("Green", "%.3f", colors.green)
                    .addData("Blue", "%.3f", colors.blue);
            telemetry.addLine()
                    .addData("Hue", "%.3f", hsvValues[0])
                    .addData("Saturation", "%.3f", hsvValues[1])
                    .addData("Value", "%.3f", hsvValues[2]);
            telemetry.addData("Alpha", "%.3f", colors.alpha);

            telemetry.addData("Starting at",  "%7d :%7d",
                    armLift.getCurrentPosition(),
                    armExtend.getCurrentPosition());
            telemetry.update();

            if(grab_mode==true) {
                telemetry.addLine("Grab Mode On");
                if (gamepad2.b)
                {
                    grab_mode = false;
                }
                if (((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM) < 2) {
                    if (hsvValues[0] > 180) {
                        telemetry.addLine("Blue!");
                    } else if (hsvValues[0] > 60) {
                        telemetry.addLine("Yellow!");
                        grabber.setPosition(0.55);
                        grab_mode=false;
                    } else {
                        telemetry.addLine("Red!");
                    }
                } else {
                    telemetry.addLine("Out of Range!");
                    grabber.setPosition(0.3);
                }
            }
            else {
                telemetry.addLine("Grab Mode Off");
                if (gamepad2.b)
                {
                    grabber.setPosition(0.3);
                }
            }
            telemetry.update();
            }
        }
    }

