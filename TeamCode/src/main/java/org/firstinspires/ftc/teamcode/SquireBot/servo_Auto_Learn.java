
package org.firstinspires.ftc.teamcode.SquireBot;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Learn", group = "Servo")
public class servo_Auto_Learn extends LinearOpMode {
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
        myBot.goToSpot(0,-21,0,2);
        sleep(500);
        myBot.goToSpot(60,-21,0,2);
        myBot.goToSpot(60,0,0,2);

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
