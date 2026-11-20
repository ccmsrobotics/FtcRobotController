package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SquireBot.Squirebot;

@Autonomous(name="Second shoot",group="Auto")
public class Autonomous_Second_shoot extends LinearOpMode {
    private Squirebot myBot;

    @Override
    public void runOpMode(){
        myBot = new Squirebot(this, hardwareMap, telemetry);
        myBot.chassis.maxSpeed = 0.7;
        myBot.GPS2.resetGPS();
        telemetry.addData("Status", "Calibrated");
        telemetry.update();
        sleep(500);


        myBot.GPS2.currentLocation(31,135,90);


        while(!isStarted()) {
            myBot.GPS2.UpdateGPS();
            // Wait for the game to start (driver presses PLAY)
            telemetry.addData("Status", "Initialized");
            telemetry.addData("X coordinate", myBot.GPS2.location.getX(DistanceUnit.INCH));
            telemetry.addData("Y coordinate", myBot.GPS2.location.getY(DistanceUnit.INCH));
            telemetry.addData("Heading angle", myBot.GPS2.location.getHeading(AngleUnit.DEGREES));
            telemetry.update();
        }

        waitForStart();
        myBot.chassis.drive(-.1, .4,0);
        sleep(500);
        myBot.chassis.drive(-1,.4,0);
        sleep(250);
        myBot.chassis.drive(0,0,0);
        myBot.goToSpot(29,116,123,0.5);
        sleep(8000);
        myBot.shooter.intakeOn();
        myBot.shooter.shooterPower=0.67;
        myBot.shooter.enableShooter();
        myBot.goToSpot(29,123,123,1.5);
        myBot.shooter.transitionOn();
        myBot.goToSpot(29,130,123,0.5);
        myBot.chassis.drive(0,.15,0);
        sleep(3000);
        myBot.shooter.intakeOff();
        myBot.shooter.disableShooter();
        myBot.shooter.transitionOff();

        //myBot.goToSpot(39,125,115,0.5);

        //sleep(5000);
        //shoot();

        myBot.goToSpot(12,120,90,0.5);


    }
    private void shoot()
    {
        myBot.shooter.shooterPower=-0.4;
        myBot.shooter.enableShooter();
        sleep(750);
        myBot.shooter.disableShooter();
        myBot.shooter.intakeOff();
        myBot.shooter.shooterPower=0.679;
        myBot.shooter.enableShooter();
        sleep(1000);
        myBot.shooter.intakePower=0.7;
        myBot.shooter.transitionPower=1;
        myBot.shooter.transitionOn();
        myBot.shooter.intakeOn();
        sleep(4000);
        myBot.shooter.disableShooter();
        myBot.shooter.intakeOff();
        myBot.shooter.transitionOff();

    }
}

