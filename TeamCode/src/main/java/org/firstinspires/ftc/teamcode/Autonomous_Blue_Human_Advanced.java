package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SquireBot.Squirebot;

@Autonomous(name="Blue Human Advanced",group="Red")
public class Autonomous_Blue_Human_Advanced extends LinearOpMode {
    private Squirebot myBot;
    private int obeliskLook;

    @Override
    public void runOpMode(){
        myBot = new Squirebot(this, hardwareMap, telemetry);
        myBot.chassis.maxSpeed = 0.7;

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

        //drive to shooting position and shoot
        myBot.goToSpot(74, 0,220,1);
        shoot();

        //pick up last row of artifacts and shoots
        myBot.goToSpot(28,-12,90,2);
        myBot.shooter.intakePower=1;
        myBot.shooter.intakeOn();
        myBot.chassis.drive(0.5,0,0);
        sleep(1800);
        myBot.chassis.drive(0,0,0);
        sleep(250);
        myBot.shooter.intakeOff();
        myBot.goToSpot(28,0,90,6);
        myBot.goToSpot(74, 0,220,1);
        shoot();

        //pick up middle row of artifacts and shoot
        myBot.goToSpot(52,-12,90,2);
        myBot.shooter.intakePower=1;
        myBot.shooter.intakeOn();
        myBot.chassis.drive(0.5,0,0);
        sleep(1800);
        myBot.chassis.drive(0,0,0);
        sleep(250);
        myBot.shooter.intakeOff();
        myBot.goToSpot(52,-14,90,6);
        myBot.goToSpot(74, 0,220,1);
        shoot();

        //move off launch line
        myBot.goToSpot(76,-12,90,1);
        myBot.shooter.intakePower=1;
        myBot.shooter.intakeOn();
        myBot.chassis.drive(0.5,0,0);
        sleep(1000);
        myBot.chassis.drive(0,0,0);
        sleep(250);
        myBot.shooter.intakeOff();

    }
    private void shoot()
    {
        myBot.shooter.intakeBackwards();
        sleep(250);
        myBot.shooter.intakeOff();
        myBot.shooter.shooterPower=0.82;
        myBot.shooter.enableShooter();
        sleep(1400);
        myBot.shooter.intakePower=0.7;
        myBot.shooter.intakeOn();
        sleep(1300);
        myBot.shooter.disableShooter();
        myBot.shooter.intakeOff();
    }
}
