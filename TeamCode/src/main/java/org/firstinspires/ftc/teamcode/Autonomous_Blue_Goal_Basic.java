package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SquireBot.Squirebot;

@Autonomous(name="Blue Goal Basic",group="Red")
public class Autonomous_Blue_Goal_Basic extends LinearOpMode {
    private Squirebot myBot;
    private int obeliskLook;

    @Override
    public void runOpMode(){
        myBot = new Squirebot(this, hardwareMap, telemetry);
        myBot.chassis.maxSpeed = 0.7;
        myBot.GPS2.resetGPS();
        blackboard.put(myBot.ALLIANCE_KEY, "BLUE");
        blackboard.put(myBot.GPS_OFFSET, -90);
        blackboard.put(myBot.X_OFFSET, 130);
        blackboard.put(myBot.Y_OFFSET, 30);
        blackboard.put(myBot.X_SCALE, -1);
        blackboard.put(myBot.Y_SCALE, 1);


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

        myBot.goToSpot(28, -24,55,1);
        myBot.shooter.intakeBackwards();
        sleep(250);
        myBot.shooter.intakeOff();
        myBot.shooter.shooterPower=0.65;
        myBot.shooter.enableShooter();
        sleep(1400);
        myBot.shooter.intakePower=0.7;
        myBot.shooter.intakeOn();
        sleep(500);
        myBot.shooter.shooterPower=0.68;
        sleep(1300);
        myBot.shooter.disableShooter();
        myBot.shooter.intakeOff();
        myBot.goToSpot(12,-20,0,2);

    }
}
