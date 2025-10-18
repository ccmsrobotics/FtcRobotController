package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SquireBot.Squirebot;


@TeleOp(name = "TeleOp Main", group = "Class")
//@Disabled
public class TeleOp_Main extends LinearOpMode {

    // Declare variables used by the class
    Squirebot myBot;
    private ElapsedTime     runtime = new ElapsedTime();
    private double headingError = 0;

    @Override
    public void runOpMode() {
        myBot = new Squirebot(this, hardwareMap, telemetry);
        myBot.chassis.maxSpeed = 0.7;

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
            myBot.camera.aprilTagRanger();
            telemetry.addLine("GoBildaData");
            telemetry.addData("X coordinate", myBot.GPS2.location.getX(DistanceUnit.INCH));
            telemetry.addData("Y coordinate", myBot.GPS2.location.getY(DistanceUnit.INCH));
            telemetry.addData("Heading angle", myBot.GPS2.location.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Shooter State", myBot.shooter.currentState);
            telemetry.addData("Shooter Power", myBot.shooter.shooterPower);
            telemetry.addData("Time left", 117-runtime.seconds());
            telemetry.addData("April tag range", myBot.camera.aprilTagRange);
            telemetry.addData("April tag bearing", myBot.camera.aprilTagBearing);



            telemetry.update();
            if(gamepad1.left_trigger<0.7) {
                myBot.chassis.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            }
            else
            {
                    myBot.chassis.driveFC(-gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x,myBot.GPS2.location.getHeading(AngleUnit.DEGREES));
            }
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
            if (gamepad1.right_trigger>0.3)
            {
                myBot.shooter.intakePower=gamepad1.right_trigger;
                myBot.shooter.intakeOn();
            } else if (gamepad2.dpad_left) {
                myBot.shooter.intakeBackwards();
            }
                else if (myBot.shooter.currentState>2)
            {


            } else
            {
                myBot.shooter.intakeOff();
            }
            myBot.shooter.shooterState(gamepad1.a||gamepad2.a,runtime.milliseconds());

            if (gamepad2.x)
            {
                myBot.shooter.shooterPower = (gamepad2.left_stick_y*0.5+0.5);
            }
            if (gamepad2.right_bumper)
            {
                myBot.shooter.shooterPower = myBot.shooter.shooterPower +.005;
                if(myBot.shooter.shooterPower > 1) myBot.shooter.shooterPower=1;
            }
            if (gamepad2.left_bumper)
            {
                myBot.shooter.shooterPower = myBot.shooter.shooterPower -.005;
                if(myBot.shooter.shooterPower < 0) myBot.shooter.shooterPower=0;
            }
            if (gamepad2.y)
            {
                myBot.shooter.openStopper();
            }
            if (gamepad2.b)
            {
                myBot.shooter.closeStopper();
            }
            if (gamepad2.dpad_up) {
                myBot.shooter.enableShooter();
            }
            if (gamepad2.dpad_down)
                myBot.shooter.disableShooter();
            if (gamepad2.dpad_left)
            {
                myBot.shooter.intakeBackwards();
            }
        }
    }
}
