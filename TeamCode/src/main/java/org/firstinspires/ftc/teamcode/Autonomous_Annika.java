package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SquireBot.Squirebot;

@Autonomous(name="Red Loading side",group="Red")
public class Autonomous_Annika extends LinearOpMode {
    private Squirebot myBot;

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

        myBot.goToSpot(72, -12,0,2);
        sleep(1500);
        myBot.goToSpot(72,-12,-45,.3);

    }
}
