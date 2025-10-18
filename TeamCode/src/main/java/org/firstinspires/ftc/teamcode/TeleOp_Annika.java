package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.teamcode.SquireBot.Squirebot;


@TeleOp(name = "ANNIKA", group = "Class")
@Disabled
public class TeleOp_Annika extends LinearOpMode {

    // Declare variables used by the class
    Squirebot myBot;
    private double headingError = 0;
    SparkFunOTOS.Pose2D pos;

    @Override
    public void runOpMode() {
        myBot = new Squirebot(this, hardwareMap, telemetry);
        myBot.chassis.maxSpeed = 0.5;
//        myBot.GPS.UpdateGPS();
//        pos = myBot.GPS.location;
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
//        telemetry.addData("X coordinate", pos.x);
//        telemetry.addData("Y coordinate", pos.y);
//        telemetry.addData("Heading angle", pos.h);
        telemetry.update();



        //Start of TeleOp
        waitForStart();
        while (opModeIsActive()) {
//            myBot.GPS.UpdateGPS();
//            pos=myBot.GPS.location;
            if(gamepad1.a)
            {
                myBot.chassis.maxSpeed = 1;
            }
            else if((gamepad1.b)){
                myBot.chassis.maxSpeed = 0.2;
            }
            else
            {
                myBot.chassis.maxSpeed = 0.5;
            }
//            myBot.chassis.driveFC(gamepad1.left_stick_y,-gamepad1.left_stick_x,-gamepad1.right_stick_x,myBot.GPS.location.h);


        }
    }
}
