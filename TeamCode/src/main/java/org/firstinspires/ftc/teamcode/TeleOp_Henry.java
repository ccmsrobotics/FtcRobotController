package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SquireBot.Squirebot;


@TeleOp(name = "teleop Class", group = "Class")
//@Disabled
public class TeleOp_Henry extends LinearOpMode {

    // Declare variables used by the class
    Squirebot myBot;
    private double headingError = 0;
    SparkFunOTOS.Pose2D pos;

    @Override
    public void runOpMode() {
        myBot = new Squirebot(this, hardwareMap, telemetry);
        myBot.chassis.maxSpeed = 0.7;
        myBot.GPS2.UpdateGPS();
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.addData("X coordinate", myBot.GPS2.location.getX(DistanceUnit.INCH));
        telemetry.addData("Y coordinate", myBot.GPS2.location.getY(DistanceUnit.INCH));
        telemetry.addData("Heading angle", myBot.GPS2.location.getHeading(AngleUnit.DEGREES));
        telemetry.update();

        double rotatorTarget = .5;

        //Start of TeleOp
        waitForStart();
        while (opModeIsActive()) {

        }
    }
}
