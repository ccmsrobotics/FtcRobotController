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

@Autonomous(name = "Auto Class", group = "Servo")
public class servo_Auto_Class extends LinearOpMode {
    SparkFunOTOS.Pose2D pos;
    private double headingError = 0;
    private Squirebot myBot;

    @Override
    public void runOpMode() {
        double drive = 0;        // Desired forward power/speed (-1 to +1)
        double strafe = 0;        // Desired strafe power/speed (-1 to +1)
        double turn = 0;        // Desired turning power/speed (-1 to +1)
        myBot = new Squirebot(this, hardwareMap, telemetry);
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

        myBot.claw.setWristPosition(.55);
        myBot.arm.startMatchAtonArm();
        myBot.arm.setArmLiftTarget(1700);
        myBot.arm.setExtendTarget(900);
        sleep(10000);
        myBot.goToSpot(8,0,0,2);
        sleep(5000);
        myBot.goToSpot(8,-10,0,2);
        sleep(5000);
        myBot.goToSpot(8,-10,90,2);
        //Move to Scoring spot
        myBot.goToSpot(8, -19, 135, 1);
        ScoreUpperBasket();

        //Pick up Second sample
        myBot.goToSpot(20.5, -15, 0, 1);
        myBot.arm.setArmLiftTarget(0);
        myBot.claw.closeGrabber();
        sleep(450);
        myBot.arm.setArmLiftTarget(1700);
        myBot.goToSpot(8, -19, 135, 1);
        ScoreUpperBasket();

        while (opModeIsActive()) {
        }
    }
//End of main loop


    private void ScoreUpperBasket() {
        // Rotate Arm
        myBot.claw.setWristPosition(.55);
        myBot.arm.setArmLiftTarget(1700);
        myBot.arm.setExtendTarget(2950);
        while (opModeIsActive() && myBot.arm.armBusy()) {
            myBot.arm.updateLocation();//
            telemetry.addData("Motor Encoders lift:extend", "%7d :%7d",
                    myBot.arm.armLiftLocation,
                    myBot.arm.armExtendLocation);
            telemetry.addData("Current Error lift:extend", "%7d :%7d",
                    (myBot.arm.armLiftTarget - myBot.arm.armLiftLocation),
                    (myBot.arm.armExtendTarget - myBot.arm.armExtendLocation));
            telemetry.update();
        }

        //Open grabber
        myBot.claw.openGrabber();
        sleep(350);

        //Move backwards so arm doesn't hit basket.  Motors are reversed, so 0.4 is move backwards
        myBot.drive.drive(-.4, 0, 0);
        myBot.arm.setExtendTarget(900);
        sleep(400);
        myBot.drive.drive(0, 0, 0);
        myBot.arm.setArmLiftTarget(100);
        myBot.claw.setWristPosition(.73);

        while (opModeIsActive() && myBot.arm.armBusy())//There is error in code.  Should be an or, but the code works so we left it.  Maybe try remove it completely.
        {
            myBot.arm.updateLocation();//
            telemetry.addData("Motor Encoders lift:extend", "%7d :%7d",
                    myBot.arm.armLiftLocation,
                    myBot.arm.armExtendLocation);
            telemetry.addData("Current Error lift:extend", "%7d :%7d",
                    (myBot.arm.armLiftTarget - myBot.arm.armLiftLocation),
                    (myBot.arm.armExtendTarget - myBot.arm.armExtendLocation));
            telemetry.update();
        }
    }


}
