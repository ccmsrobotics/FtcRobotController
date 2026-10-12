package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SquireBot.Squirebot;


@TeleOp(name = "Drive Test", group = "Class")
//@Disabled
public class driveTest extends LinearOpMode {

    // Declare variables used by the class
    Squirebot myBot;
    private ElapsedTime runtime = new ElapsedTime();
    private double headingError = 0;
    private double targetAngle = 0;
    private String alliance;
    int resetDebounce = 0;
    int kickDebounce = 0;
    double driveAngle, shootAngle;
    double calcXOffset, calcYOffset;

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

        //Start of TeleOp
        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            myBot.GPS2.UpdateGPS();
            driveAngle = myBot.GPS2.location.getHeading(AngleUnit.DEGREES);
            shootAngle = -driveAngle;
            while (driveAngle > 180) driveAngle -= 360;
            while (driveAngle <= -180) driveAngle += 360;
            while (shootAngle > 180) shootAngle -= 360;
            while (shootAngle <= -180) shootAngle += 360;

            //calculate target angle
            if (myBot.GPS2.location.getX(DistanceUnit.INCH) < 48) {
                targetAngle = Math.toDegrees(Math.atan2((60 - myBot.GPS2.location.getY(DistanceUnit.INCH)), (48 - myBot.GPS2.location.getY(DistanceUnit.INCH))));
            } else if (myBot.GPS2.location.getX(DistanceUnit.INCH) > 96) {
                targetAngle = Math.toDegrees(Math.atan2((60 - myBot.GPS2.location.getY(DistanceUnit.INCH)), (96 - myBot.GPS2.location.getY(DistanceUnit.INCH))));
            } else {
                targetAngle = driveAngle;
            }

            telemetry.addLine("GoBildaData");
            telemetry.addData("X coordinate", myBot.GPS2.location.getX(DistanceUnit.INCH));
            telemetry.addData("Y coordinate", myBot.GPS2.location.getY(DistanceUnit.INCH));
            telemetry.addData("Heading angle", shootAngle);
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Shooter State", myBot.shooter.currentState);
            telemetry.addData("Shooter Power", myBot.shooter.shooterPower);
            telemetry.addData("Calc X offset", calcXOffset);
            telemetry.addData("Calc Y offset", calcYOffset);
            telemetry.addData("Kickstand Encoder", myBot.chassis.kickStand.getCurrentPosition());

            telemetry.addData("Time left", 117 - runtime.seconds());
            //telemetry.addData("April tag range", myBot.camera.aprilTagRange);
            //telemetry.addData("April tag bearing", myBot.camera.aprilTagBearing);


            telemetry.update();
            if (gamepad1.a) {
                myBot.chassis.leftFrontDrive.setPower(0.3);
            }
            if (gamepad1.b) {
                myBot.chassis.rightFrontDrive.setPower(0.3);
            }
            if (gamepad1.y) {
                myBot.chassis.leftBackDrive.setPower(0.3);
            }
        }

    }
}
