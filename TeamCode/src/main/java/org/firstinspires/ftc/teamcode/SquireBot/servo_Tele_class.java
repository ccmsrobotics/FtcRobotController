
package org.firstinspires.ftc.teamcode.SquireBot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "teleop Class", group = "Class")
//@Disabled
public class servo_Tele_class extends LinearOpMode {

    // Declare variables used by the class
    Squirebot myBot;
    private double headingError = 0;
    SparkFunOTOS.Pose2D pos;

    @Override
    public void runOpMode() {
        myBot = new Squirebot(this, hardwareMap, telemetry);
        myBot.drive.maxSpeed = 0.7;
        myBot.GPS.UpdateGPS();
        pos = myBot.GPS.location;
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        commonTelemetry();

        double rotatorTarget = .5;

        //Start of TeleOp
        waitForStart();
        myBot.startTele();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            myBot.GPS.UpdateGPS();
            pos = myBot.GPS.location;
            myBot.arm.updateEncoders();//avoid repeated calls to read encoders to reduce loop time

            if (gamepad1.back) {
                resetCompass();
            }
            //Arm Lift
            if (Math.abs(gamepad2.left_stick_y) > .15) {
                myBot.arm.setArmLiftTarget(myBot.arm.armLiftTarget - Math.round(gamepad2.left_stick_y * 12));
            }

            if (gamepad2.dpad_up)
                myBot.arm.setArmLiftTarget(1700);
            else if (gamepad2.dpad_right) {
                if (myBot.arm.armExtendLocation < 1000) {
                    myBot.arm.setArmLiftTarget(120);
                    rotatorTarget = 0.41;
                }
            } else if (gamepad2.dpad_down) {
                if (myBot.arm.armExtendLocation < 1900)
                    myBot.arm.setArmLiftTarget(0);
            } else if (gamepad2.dpad_left) {
                rotatorTarget = 0.56;
                myBot.arm.setArmLiftTarget(1174);

            } else if (gamepad2.right_trigger > 0.5) {
                rotatorTarget = 0.33;
                myBot.arm.setArmLiftTarget(775);

            }
            //ARM EXTEND
            if (gamepad2.a) {
                myBot.arm.setExtendTarget(myBot.arm.armExtendTarget + 25);
            } else if (gamepad2.b) {
                myBot.arm.setExtendTarget(myBot.arm.armExtendTarget - 25);
            }


            //rotator location
            if (gamepad2.x)
                myBot.claw.setWristPosition(0.73);
            else if (gamepad2.y)
                myBot.claw.setWristPosition(0.5);

            if (gamepad1.left_bumper)
                myBot.drive.maxSpeed = 0.2;
            else if (gamepad1.right_bumper) {
                myBot.drive.maxSpeed = 1;
            } else {
                myBot.drive.maxSpeed = 0.7;
            }
            myBot.drive.driveFC(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, myBot.GPS.location.h);

            myBot.claw.setWristPosition(rotatorTarget);
            if (gamepad1.left_trigger > 0.5) {
                myBot.claw.closeGrabber();
            } else if (gamepad1.right_trigger > 0.5) {
                myBot.claw.openGrabber();
            }
         commonTelemetry();
        }
    }

    private void commonTelemetry(){
        telemetry.addData("X coordinate", pos.x);
        telemetry.addData("Y coordinate", pos.y);
        telemetry.addData("Heading angle", pos.h);
        telemetry.addData("Motor Encoders lift:extend", "%7d :%7d",
                myBot.arm.armLiftLocation,
                myBot.arm.armExtendLocation);
        telemetry.addData("Current Error lift:extend", "%7d :%7d",
                (myBot.arm.armLiftTarget - myBot.arm.armLiftLocation),
                (myBot.arm.armExtendTarget - myBot.arm.armExtendLocation));
        telemetry.update();
    }

    private void resetCompass() {
        telemetry.addData("Reseting IMU", "wait");
        telemetry.update();
        myBot.drive.drive(0, 0, 0);
        sleep(150);
        myBot.GPS.resetGPS();
    }


}
