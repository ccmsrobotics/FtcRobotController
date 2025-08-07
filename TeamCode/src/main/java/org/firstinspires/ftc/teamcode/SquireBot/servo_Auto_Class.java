/* Copyright (c) 2023 FIRST. All rights reserved.
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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="Auto 4 top basket to Ascend", group = "Servo")
public class servo_Auto_Class extends LinearOpMode
{

    SparkFunOTOS.Pose2D pos;
    private double headingError  = 0;
    private Squirebot myBot;
    @Override public void runOpMode()
    {
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)
        myBot = new Squirebot(HardwareMap,telemetry);
        //Arm (rotation and extend config)
        myBot.arm.resetArm();
        sleep(1000);
        myBot.arm.resetEncoders();
        pos = myBot.GPS.GPS;
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.addData("X coordinate", pos.x);
        telemetry.addData("Y coordinate", pos.y);
        telemetry.addData("Heading angle", pos.h);
        telemetry.update();
        myBot.claw.closeGrabber();
        myBot.claw.setWristPosition(.15);

        //Wait for start
        waitForStart();
        myOtos.resetTracking();
        //Move arm to driving location
        rotator.setPosition(.55);
        armLift.setPower(1);
        armExtend.setPower(1);
        armLift.setTargetPosition(1700);
        armExtend.setTargetPosition(900);
        //Move to Scoring spot
        goToSpot(8,-19,135,1);
        ScoreUpperBasket();

        //Pick up Second sample
        goToSpot(20.5,-15,0,1);
        armLift.setTargetPosition(0);
        grabber.setPosition(0.02);
        sleep(450);
        armLift.setTargetPosition(1700);
        goToSpot(8,-19,135,1);
        ScoreUpperBasket();

        //Pick up and score third sample
        goToSpot(20.5,-25,0,1);
        armLift.setTargetPosition(0);
        grabber.setPosition(0.02);
        sleep(450);
        armLift.setTargetPosition(1700);
        goToSpot(8,-19,135,1);
        ScoreUpperBasket();

        //Pickup fourth
        armLift.setTargetPosition(300);
        goToSpot(16.5,-22,34,0.5);
        armExtend.setTargetPosition(1800);
        sleep(350);
        armLift.setTargetPosition(150);
        //sleep(210);
        sleep(550);
        grabber.setPosition(0.02);
        sleep(450);
        armExtend.setTargetPosition(900);
        armLift.setTargetPosition(1700);
        goToSpot(8,-19,135,1);
        ScoreUpperBasket();

        //Move to Teleop start position - This may be updated to Lvl 1 ascend, but will require not resetting IMU and motor encoders.
        armLift.setTargetPosition(1000);
        rotator.setPosition(0.1);
        goToSpot(50,-4,-90,1);
        armExtend.setTargetPosition(1657);
        goToSpot(50,4.25,-90,1);
        armLift.setPower(0);
        //allow gravity to move arm to bar
        rotator.setPosition(-1);

        while (opModeIsActive())
        {
        }
    }
//End of main loop


    private void ScoreUpperBasket()
    {
        // Rotate Arm
        rotator.setPosition(WRISTUPPERBASKET);//would this dropoff easier if we didnt change?
        armLift.setTargetPosition(1700);
        //sleep(250); //Is this necessary?  Delete?
        // Extend arm
        armExtend.setTargetPosition(2950);
        while (opModeIsActive() && (armExtend.isBusy() && armLift.isBusy()))
        {
            telemetry.addData("Extending arms", " at %7d :%7d",
                    armLift.getCurrentPosition(), armExtend.getCurrentPosition());
            telemetry.update();
        }
        while(opModeIsActive()&& armExtend.getCurrentPosition() <2850)
        {}
        //Open grabber
        grabber.setPosition(.40);
        sleep(350);

        //Move backwards so arm doesn't hit basket.  Motors are reversed, so 0.4 is move backwards
        moveRobot(.4,0,0);
        armExtend.setTargetPosition(900);
        sleep(400);
        moveRobot(0,0,0);
        armLift.setTargetPosition(100);
        rotator.setPosition(.73);

        while (opModeIsActive() && (armExtend.isBusy() && armLift.isBusy()))//There is error in code.  Should be an or, but the code works so we left it.  Maybe try remove it completely.
        {

            // Display it for the driver.
            telemetry.addData("Extending arms", " at %7d :%7d",
                    armLift.getCurrentPosition(), armExtend.getCurrentPosition());
            telemetry.update();
        }
    }


}
