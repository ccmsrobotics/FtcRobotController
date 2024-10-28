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

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name="Servo Omni grabber field centric", group="Linear OpMode")
//@Disabled
public class servo_omnidrive_fc extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private Servo grabber = null;
    private Servo rotator = null;
    private boolean grab_mode = false;
    NormalizedColorSensor colorSensor;
    private DcMotor armLift = null;
    private DcMotor armExtend = null;
    private double armLiftPower;
    private double armExtendPower;
    SparkFunOTOS myOtos;
    SparkFunOTOS.Pose2D pos;

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
        grabber = hardwareMap.get(Servo.class, "grabber");
        rotator = hardwareMap.get(Servo.class, "rotator");
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
        armLift.setTargetPosition(0);
        armLift.setPower(0);
        armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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


        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        configureOtos();
        pos = myOtos.getPosition();
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.addData("X coordinate", pos.x);
        telemetry.addData("Y coordinate", pos.y);
        telemetry.addData("Heading angle", pos.h);
        telemetry.update();
        int armLiftLocation;
        int armExtendLocation;
        int armLiftTarget=0;
        int armExtendTarget;
        double rotatorTarget=.5;
        double speedMode = .7;
        grabber.setPosition(0.0);
        rotator.setPosition(0.2);
        waitForStart();
        armLift.setPower(1);
        colorSensor.setGain(gain);
        runtime.reset();
        rotator.setPosition(.5);
        grabber.setPosition(0);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //fix compass heading
            if (gamepad1.options) {
                telemetry.addData("Reseting IMU", "wait");
                telemetry.update();
                resetCompass();
            }

            //ARM LIFT

            armLiftLocation = armLift.getCurrentPosition();
            armExtendLocation = armExtend.getCurrentPosition();
            //armLift.setPower(armLiftSpeed);
            armLiftPower = gamepad2.left_stick_y;
            if (gamepad2.right_stick_y < -.75) {
                if (armLiftTarget < 1650)
                    armLiftTarget = armLiftTarget + 2;
            }
            if (gamepad2.right_stick_y > .75) {
                if (armLiftTarget > 0)
                    armLiftTarget = armLiftTarget - 4;
            }

            if (gamepad2.dpad_up)
                armLiftTarget =1550;
            else if (gamepad2.dpad_right) {
                if (armExtendLocation < 1900)
                    armLiftTarget = 400;
            }
                else if (gamepad2.dpad_down) {
                    if (armExtendLocation < 1900)
                        armLiftTarget = 0;
            }
            //ARM EXTEND

            if (armLiftLocation < 600) {
                if (armExtendLocation < 100) {
                    if (gamepad2.a)
                        armExtendPower = 1;
                    else
                        armExtendPower = 0;
                } else if (armExtendLocation < 1900) {
                    if (gamepad2.a)
                        armExtendPower = 1;
                    else if (gamepad2.b)
                        armExtendPower = -1;
                    else
                        armExtendPower = 0;
                } else {
                    if (gamepad2.b)
                        armExtendPower = -1;
                    else
                        armExtendPower = 0;
                }
            } else if (armLiftLocation > 600) {
                if (armExtendLocation < 100) {
                    if (gamepad2.a)
                        armExtendPower = 1;
                    else
                        armExtendPower = 0;
                } else if (armExtendLocation < 2750) {
                    if (gamepad2.a)
                        armExtendPower = 1;
                    else if (gamepad2.b)
                        armExtendPower = -1;
                    else
                        armExtendPower = 0;
                } else if (armExtendLocation < 2900) {
                    if (gamepad2.a)
                        armExtendPower = 0.5;
                    else if (gamepad2.b)
                        armExtendPower = -0.5;
                    else
                        armExtendPower = 0;
                } else {
                    if (gamepad2.b)
                        armExtendPower = -1;
                    else
                        armExtendPower = 0;
                }
            }

            //rotator location
            if (gamepad2.x)
                rotatorTarget =0.73;
            else if (gamepad2.y)
                rotatorTarget =0.5;



            double max;
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = -gamepad1.left_stick_x;
            double yaw = -gamepad1.right_stick_x;
            if (gamepad1.left_bumper)
                speedMode = 0.2;
            else if (gamepad1.right_bumper) {
                speedMode = 1;
            } else {
                speedMode = 0.7;
            }

            if (gamepad2.x) {
                grab_mode = true;
            }
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            pos = myOtos.getPosition();
            double currentYawRadians = pos.h*3.1415/180;
            double rotStrafe = lateral * Math.cos(-currentYawRadians) - axial * Math.sin(-currentYawRadians);
            double rotDrive = lateral * Math.sin(-currentYawRadians) + axial * Math.cos(-currentYawRadians);
            rotStrafe = rotStrafe*1.1;
            double leftFrontPower = rotDrive + rotStrafe + yaw;
            double rightFrontPower = rotDrive - rotStrafe - yaw;
            double leftBackPower = rotDrive - rotStrafe + yaw;
            double rightBackPower = rotDrive + rotStrafe - yaw;

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

                leftFrontPower *= speedMode;
                rightFrontPower *= speedMode;
                leftBackPower *= speedMode;
                rightBackPower *= speedMode;


                // Send calculated power to wheels
                leftFrontDrive.setPower(leftFrontPower);
                rightFrontDrive.setPower(rightFrontPower);
                leftBackDrive.setPower(leftBackPower);
                rightBackDrive.setPower(rightBackPower);
                armExtend.setPower(armExtendPower);
                armLift.setTargetPosition(armLiftTarget);
                rotator.setPosition(rotatorTarget);
                if (gamepad2.right_bumper){
                    grabber.setPosition(0);
                } else if (gamepad2.left_bumper) {
                    grabber.setPosition(.3);
                }

                // Show the elapsed game time and wheel power.
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
                telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);


                telemetry.addData("Starting at", "%7d :%7d",
                        armLift.getCurrentPosition(),
                        armExtend.getCurrentPosition());
                telemetry.update();


            }
        }
        private void resetCompass(){
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);
            armExtend.setPower(0);
            sleep(150);
            myOtos.calibrateImu();
        }

    private void configureOtos() {
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

        // Set the desired units for linear and angular measurements. Can be either
        // meters or inches for linear, and radians or degrees for angular. If not
        // set, the default is inches and degrees. Note that this setting is not
        // persisted in the sensor, so you need to set at the start of all your
        // OpModes if using the non-default value.
        // myOtos.setLinearUnit(DistanceUnit.METER);
        myOtos.setLinearUnit(DistanceUnit.INCH);
        // myOtos.setAngularUnit(AnguleUnit.RADIANS);
        myOtos.setAngularUnit(AngleUnit.DEGREES);

        // Assuming you've mounted your sensor to a robot and it's not centered,
        // you can specify the offset for the sensor relative to the center of the
        // robot. The units default to inches and degrees, but if you want to use
        // different units, specify them before setting the offset! Note that as of
        // firmware version 1.0, these values will be lost after a power cycle, so
        // you will need to set them each time you power up the sensor. For example, if
        // the sensor is mounted 5 inches to the left (negative X) and 10 inches
        // forward (positive Y) of the center of the robot, and mounted 90 degrees
        // clockwise (negative rotation) from the robot's orientation, the offset
        // would be {-5, 10, -90}. These can be any value, even the angle can be
        // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0.55, -5.55, 0);
        myOtos.setOffset(offset);

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        myOtos.setLinearScalar(1.0);
        myOtos.setAngularScalar(1.0);

        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. Note that as of firmware version 1.0, the calibration
        // will be lost after a power cycle; the OTOS performs a quick calibration
        // when it powers up, but it is recommended to perform a more thorough
        // calibration at the start of all your OpModes. Note that the sensor must
        // be completely stationary and flat during calibration! When calling
        // calibrateImu(), you can specify the number of samples to take and whether
        // to wait until the calibration is complete. If no parameters are provided,
        // it will take 255 samples and wait until done; each sample takes about
        // 2.4ms, so about 612ms total
        myOtos.calibrateImu();

        // Reset the tracking algorithm - this resets the position to the origin,
        // but can also be used to recover from some rare tracking errors
        myOtos.resetTracking();

        // After resetting the tracking, the OTOS will report that the robot is at
        // the origin. If your robot does not start at the origin, or you have
        // another source of location information (eg. vision odometry), you can set
        // the OTOS location to match and it will continue to track from there.
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        myOtos.getVersionInfo(hwVersion, fwVersion);

        telemetry.addLine("OTOS configured! Press start to get position data!");
        telemetry.addLine();
        telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
        telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
        telemetry.update();
    }

}
