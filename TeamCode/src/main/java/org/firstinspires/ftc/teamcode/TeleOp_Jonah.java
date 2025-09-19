package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SquireBot.Squirebot;


@TeleOp(name = "Jonah", group = "Class")
//@Disabled
public class TeleOp_Jonah extends LinearOpMode {

    // Declare variables used by the class
    Squirebot myBot;
    private double headingError = 0;
    SparkFunOTOS.Pose2D pos;

    @Override
    public void runOpMode() {
        myBot = new Squirebot(this, hardwareMap, telemetry);
        myBot.chassis.maxSpeed = 0.7;
        myBot.GPS.UpdateGPS();
        pos = myBot.GPS.location;
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.addData("X coordinate", pos.x);
        telemetry.addData("Y coordinate", pos.y);
        telemetry.addData("Heading angle", pos.h);
        telemetry.update();

        //Start of TeleOp
        waitForStart();
        while (opModeIsActive()) {
            myBot.GPS.UpdateGPS();
            myBot.GPS2.UpdateGPS();
            pos = myBot.GPS.location;
            telemetry.addData("SparkFun", "Data");
            telemetry.addData("X coordinate", pos.x);
            telemetry.addData("Y coordinate", pos.y);
            telemetry.addData("Heading angle", pos.h);
            telemetry.addLine("GoBildaData");
            telemetry.addData("X coordinate", myBot.GPS2.location.getX(DistanceUnit.INCH));
            telemetry.addData("Y coordinate", myBot.GPS2.location.getY(DistanceUnit.INCH));
            telemetry.addData("Heading angle", myBot.GPS2.location.getHeading(AngleUnit.DEGREES));

            telemetry.update();
            myBot.chassis.drive(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            if (gamepad1.left_bumper) {
                myBot.chassis.maxSpeed = 0.3;
            }
            else if (gamepad1.right_bumper)
            {
                myBot.chassis.maxSpeed = 1;
            }
            else
            {
                myBot.chassis.maxSpeed = 0.7;
            }
        }
    }
}
