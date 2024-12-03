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

package org.firstinspires.ftc.teamcode.RoboSquires2024;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

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

@TeleOp(name="Foot", group="Iterative OpMode")
@Disabled
public class BasicOpMode_Iterative extends OpMode
{
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

    int seesawPosition = 0;
    int seesawSensitivity = 50;
    int extenderBasePosition = 0;
    int seesawTiltPosition = 2500;
    double clawPosition = 0;
    int extenderpos = 0;
    double seesawPower = 0;
    double wristPosition = 0;

    // robot mode
    // enter power saver mode if NOT in preset and extender is retracted
    // enter preset mode when preset button is pressed, do not exit unless
    //   seesaw or extender is manually manipulated by the driver
    // enter classic
    //   when NOT in preset mode and extender is extended
    //   OR when seesaw or extender is manually manipulated by the driver
    enum Mode {
        powersaver,preset,classic
    }
    // start in powersaver mode
    Mode robotMode = Mode.powersaver;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    //@Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        topleftDrive  = hardwareMap.get(DcMotor.class, "top_left_drive");
        toprightDrive = hardwareMap.get(DcMotor.class, "top_right_drive");
        backleftDrive  = hardwareMap.get(DcMotor.class, "back_left_drive");
        backrightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        extender = hardwareMap.get(DcMotor.class,"extender");
        seesaw = hardwareMap.get(DcMotor.class, "seesaw");
        seesawPosition = seesaw.getCurrentPosition();
        extenderBasePosition = extender.getCurrentPosition();
        seesawTiltPosition = extenderBasePosition + 2500;
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
        seesaw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        clawPosition = claw.getPosition();
        claw.setPosition(1); // used to be 0
        wrist.setPosition(wristPosition);
        seesaw.setTargetPosition(0);
        seesaw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extender.setTargetPosition(0);
        extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */



    @Override
    public void loop() {

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double max;
        double axial = -gamepad1.left_stick_y / 1.5;  // Note: pushing stick forward gives negative value
        double lateral = gamepad1.left_stick_x / 1.5;
        double yaw = gamepad1.right_stick_x / 2;

        seesawPower = 0;

        // only switch into powersave or classic if NOT moving to a preset
        if (robotMode != Mode.preset) {
            if (extender.getCurrentPosition() > seesawTiltPosition) {
                robotMode = Mode.classic;
            }
            else {
                robotMode = Mode.powersaver;
            }
        }

        if (gamepad2.left_trigger > 0
                && extender.getCurrentPosition() < (5600 + extenderBasePosition)) {
            extenderpos = extender.getCurrentPosition() + 100;
            robotMode = Mode.classic;
        }
        if (gamepad2.right_trigger > 0
                && extender.getCurrentPosition() > extenderBasePosition) {
            extenderpos = extender.getCurrentPosition() - 100;
            robotMode = Mode.classic;
        }

        if (robotMode == Mode.powersaver) {
            if (gamepad2.dpad_up) {
                seesawPower = 0.3;
            }
            if (gamepad2.dpad_down) {
                seesawPower = -0.3;
            }
        }
        else {
            if (gamepad2.dpad_up && (seesawPosition - seesaw.getCurrentPosition()) < seesawSensitivity) {
                seesawPosition += seesawSensitivity;
                robotMode = Mode.classic;
            }
            if (gamepad2.dpad_down && (seesawPosition - seesaw.getCurrentPosition()) > -seesawSensitivity) {
                seesawPosition -= seesawSensitivity;
                robotMode = Mode.classic;
            }
        }

        if (gamepad2.x) {
            seesawPosition = -1000;
            extenderpos = 3000 + extenderBasePosition;
            wristPosition = 0.92;
            clawPosition = 0;
            robotMode = Mode.preset;
        }
        if (gamepad2.y) {
            extenderpos = 5000 + extenderBasePosition;
            seesawPosition = 800;
            wristPosition = 0;
            robotMode = Mode.preset;
        }

        if (gamepad2.a && clawPosition < 1.0) {
            clawPosition += 0.01;
        }
        if (gamepad2.b && clawPosition > 0.0) {
            clawPosition -= 0.01;
        }

        if (gamepad2.left_bumper && wristPosition < 1) {
            wristPosition += 0.01;
        }
        if (gamepad2.right_bumper && wristPosition > 0) {
            wristPosition -= 0.01;
        }

        if (gamepad1.right_trigger > 0) {
            lateral = gamepad1.right_trigger / 1.5;
        }
        if (gamepad1.left_trigger > 0) {
            lateral = gamepad1.left_trigger / -1.5;
        }

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
        topleftDrive.setPower(leftFrontPower);
        toprightDrive.setPower(rightFrontPower);
        backleftDrive.setPower(leftBackPower);
        backrightDrive.setPower(rightBackPower);

        extender.setTargetPosition(extenderpos);
        extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // set the power of the extender when it is not in position, otherwise
        // set the power to 0. Intent is to save motor power
        if (extender.getCurrentPosition() > extenderpos + 5 ||
                extender.getCurrentPosition() < extenderpos - 5) {
            extender.setPower(0.9);
        }
        else {
            extender.setPower(0);
        }

        if (robotMode == Mode.powersaver) {
            seesaw.setPower(seesawPower);
            seesaw.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        else { // if in classic or preset, run to position
            seesaw.setPower(0.25);
            seesaw.setTargetPosition(seesawPosition);
            seesaw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        claw.setPosition(clawPosition);
        wrist.setPosition(wristPosition);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Wrist Position (actual)", wrist.getPosition());
        telemetry.addData("Wrist Position (program)", wristPosition);
        telemetry.addData("Claw Position", claw.getPosition());
        telemetry.addData("Seesaw Target", seesawPosition);
        telemetry.addData("Seesaw Position", seesaw.getCurrentPosition());
        telemetry.addData("Extender target", extenderpos);
        telemetry.addData("Extender Position", extender.getCurrentPosition());
        telemetry.addData("Mode", robotMode);
        telemetry.addData("Tilt Position", seesawTiltPosition);
    }


        /*
         * Code to run ONCE after the driver hits STOP
         */
        @Override
        public void stop() {
        }

}
