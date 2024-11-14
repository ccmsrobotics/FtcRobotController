/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous
//@Disabled
public class BasicOpMode_Autonomous_1150 extends LinearOpMode
{
    double speedMultiplier = 1.0;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor topleftDrive = null;
    private DcMotor toprightDrive = null;
    private DcMotor backleftDrive = null;
    private DcMotor backrightDrive = null;

    private DcMotor extender = null;
    private DcMotor seesaw = null;
    private Servo claw = null;
    private Servo wrist = null;

    // time between movements
    private int cooldownTime = 100;
    int baseSeesaw;
    int baseExtender;

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */


    double forward = 0.5;
    double backward = -0.5;
    double right = 0.5;
    double left = -0.5;

    @Override
    public void runOpMode() {


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        topleftDrive  = hardwareMap.get(DcMotor.class, "top_left_drive");
        toprightDrive = hardwareMap.get(DcMotor.class, "top_right_drive");
        backleftDrive  = hardwareMap.get(DcMotor.class, "back_left_drive");
        backrightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        extender = hardwareMap.get(DcMotor.class,"extender");
        seesaw = hardwareMap.get(DcMotor.class, "seesaw");
        claw = hardwareMap.get(Servo.class,"claw");
        wrist = hardwareMap.get(Servo.class,"wrist");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        topleftDrive.setDirection(DcMotor.Direction.FORWARD);
        toprightDrive.setDirection(DcMotor.Direction.FORWARD);
        backleftDrive.setDirection(DcMotor.Direction.FORWARD);
        backrightDrive.setDirection(DcMotor.Direction.FORWARD);

        extender.setDirection(DcMotor.Direction.FORWARD);
        seesaw.setDirection(DcMotorSimple.Direction.REVERSE);

        seesaw.setTargetPosition(baseSeesaw);
        seesaw.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        extender.setTargetPosition(baseExtender);
        extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        telemetry.addData("Status", "Starting...");

        telemetry.addData("Seesaw", seesaw.getCurrentPosition());
        telemetry.addData("Extender", extender.getCurrentPosition());
        telemetry.addData("Claw", claw.getPosition());
        telemetry.addData("Wrist", wrist.getPosition());
        telemetry.update();

        // Code for autonomous mode

        baseExtender = extender.getCurrentPosition();
        baseSeesaw = seesaw.getCurrentPosition();

        // set the seesaw position
        setSeesawPosition(baseSeesaw + 1500,2000);
        // extend the arm
        longyarm(baseExtender + 5000,2000);
        telemetry.update();
        setWristPosition(0);
        // forward
        moveRobotVertical(1150,forward);
        telemetry.update();
        // retract extender
        longyarm(baseExtender + 4000,2000);
        longyarm(baseExtender + 2500,2000);
        telemetry.update();
        // open the claw
        setClawPosition(0.5);
        sleep(500);

        moveRobotVertical(1150,backward);
        longyarm(baseExtender,1000);
        setSeesawPosition(baseSeesaw,1000);
        moveRobotLateral(1900,right);
        moveRobotVertical(2000,forward);
        moveRobotLateral(400,right);
        moveRobotVertical(1850,backward);
    }
    // Positive power moves the robot forward
    private void moveRobotVertical(int pTime, double pPower) {

        double lPower = pPower * speedMultiplier;

        topleftDrive.setPower(lPower);
        toprightDrive.setPower(lPower);
        backleftDrive.setPower(lPower);
        backrightDrive.setPower(lPower);

        sleep(pTime);

        topleftDrive.setPower(0);
        toprightDrive.setPower(0);
        backleftDrive.setPower(0);
        backrightDrive.setPower(0);

        sleep(cooldownTime);
    }

    // positive power moves the robot right
    private void moveRobotLateral(int pTime, double pPower) {

        double lPower = pPower * speedMultiplier;

        topleftDrive.setPower(lPower);
        toprightDrive.setPower(-lPower);
        backleftDrive.setPower(-lPower);
        backrightDrive.setPower(lPower);

        sleep(pTime);

        topleftDrive.setPower(0);
        toprightDrive.setPower(0);
        backleftDrive.setPower(0);
        backrightDrive.setPower(0);

        sleep(cooldownTime);
    }

    // Positive moves the extender out
    private void longyarm(int position, int time){
        int offsetExtender = baseExtender + position;
        extender.setTargetPosition(offsetExtender);
        extender.setPower(0.9);

        sleep(time);
    }

    // TBD (seesaw)
    private void setSeesawPosition(int position, int time) {
        int offsetSeesaw = baseSeesaw + position;
        seesaw.setTargetPosition(offsetSeesaw);
        seesaw.setPower(0.6);

        sleep(time);
    }

    // TBD
    private void setClawPosition(double position) {

        claw.setPosition(position);
    }

    // TBD
    private void setWristPosition(double position) {

        wrist.setPosition(position);
    }
}
