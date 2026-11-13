package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SquireBot.Squirebot;

@Autonomous(name="First shoot",group="Auto")
public class Autonomous_First_shoot extends LinearOpMode {
    private Squirebot myBot;

    @Override
    public void runOpMode(){
        myBot = new Squirebot(this, hardwareMap, telemetry);
        myBot.chassis.maxSpeed = 0.7;
        myBot.GPS2.resetGPS();
        telemetry.addData("Status", "Calibrated");
        telemetry.update();
        sleep(500);


        myBot.GPS2.currentLocation(31,9,-90);


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
        myBot.chassis.drive(-1,.4,0);
        sleep(500);
        myBot.chassis.drive(0,0,0);
        myBot.goToSpot(40,38,-90,0.5);
        myBot.shooter.intakeOn();
        myBot.goToSpot(40,20,-90,0.5);
        sleep(1200);
        myBot.shooter.intakeOff();

        myBot.goToSpot(39,16,-115,0.5);
        shoot();


        myBot.goToSpot(36,24,-90,0.5);
        myBot.goToSpot(24,24,-90,0.5);
        myBot.goToSpot(12,90,-90,0.5);
        //How long does it take?________
        //myBot.goToSpot(100,9,90,1);
        /*
        myBot.goToSpot(28, -24,55,1);
        shoot();


        myBot.goToSpot(50,-10,-90,2);
        myBot.shooter.intakePower=1;
        myBot.shooter.intakeOn();
        myBot.chassis.drive(0.5,0,0);
        sleep(1400);
        myBot.chassis.drive(0,0,0);
        sleep(250);
        myBot.shooter.intakeOff();

        myBot.goToSpot(58,4,-45,2);
        myBot.chassis.drive(.5,.5,0);
        sleep(2000);

        myBot.goToSpot(28, -24,55,1);
        shoot();

        myBot.goToSpot(74,-10,-90,2);
        myBot.shooter.intakePower=1;
        myBot.shooter.intakeOn();
        myBot.chassis.drive(0.5,0,0);
        sleep(1400);
        myBot.chassis.drive(0,0,0);
        sleep(250);
        myBot.shooter.intakeOff();

        myBot.goToSpot(28, -24,55,1);
        shoot();

        myBot.goToSpot(12,-20,0,2); */

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

