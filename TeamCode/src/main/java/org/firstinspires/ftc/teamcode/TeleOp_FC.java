package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SquireBot.Squirebot;


@TeleOp(name = "TeleOp Field Centric", group = "Class")
//@Disabled
public class TeleOp_FC extends LinearOpMode {

    // Declare variables used by the class
    Squirebot myBot;
    private ElapsedTime     runtime = new ElapsedTime();
    private double headingError = 0;
    private double targetAngle=0;
    private String alliance;
    int resetDebounce = 0;
    double driveAngle, shootAngle;
    double calcXOffset,calcYOffset;

    @Override
    public void runOpMode() {
        myBot = new Squirebot(this, hardwareMap, telemetry);
        myBot.chassis.maxSpeed = 0.7;
        alliance = (String) blackboard.getOrDefault(myBot.ALLIANCE_KEY,"Purple");
        myBot.GPS2.UpdateGPS();

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.addData("X coordinate", myBot.GPS2.location.getX(DistanceUnit.INCH));
        telemetry.addData("Y coordinate", myBot.GPS2.location.getY(DistanceUnit.INCH));
        telemetry.addData("Heading angle", myBot.GPS2.location.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Alliance", alliance);
        double FC_offset = ((Number) blackboard.getOrDefault(myBot.GPS_OFFSET,0)).doubleValue();
        double xOffset = ((Number) blackboard.getOrDefault(myBot.X_OFFSET,0)).doubleValue();
        double yOffset = ((Number) blackboard.getOrDefault(myBot.Y_OFFSET,0)).doubleValue();
        double xScale = ((Number) blackboard.getOrDefault(myBot.X_SCALE,1)).doubleValue();
        double yScale = ((Number) blackboard.getOrDefault(myBot.Y_SCALE,1)).doubleValue();

        telemetry.addData("GPS_OFFSET", FC_offset);
        telemetry.update();

        //Start of TeleOp
        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            myBot.GPS2.UpdateGPS();
            driveAngle = myBot.GPS2.location.getHeading(AngleUnit.DEGREES)+FC_offset;
            shootAngle = -driveAngle;
            while (driveAngle > 180) driveAngle -= 360;
            while (driveAngle <= -180) driveAngle += 360;
            while (shootAngle > 180) shootAngle -= 360;
            while (shootAngle <= -180) shootAngle += 360;

            //calculate target angle
            if(alliance=="RED") {
                calcXOffset = (142 - myBot.GPS2.location.getX(DistanceUnit.INCH) * xScale - xOffset);
                calcYOffset = yOffset + yScale * myBot.GPS2.location.getY(DistanceUnit.INCH);
                targetAngle = -Math.toDegrees(Math.atan2((142 - myBot.GPS2.location.getX(DistanceUnit.INCH) * xScale - xOffset), yOffset + yScale * myBot.GPS2.location.getY(DistanceUnit.INCH)));
            }
                else {
                    calcYOffset = yOffset + yScale * myBot.GPS2.location.getY(DistanceUnit.INCH);
                    calcXOffset = 142 - myBot.GPS2.location.getX(DistanceUnit.INCH) * xScale - xOffset;
                targetAngle = Math.toDegrees(Math.atan2((142 - myBot.GPS2.location.getX(DistanceUnit.INCH) * xScale - xOffset), yOffset + yScale * myBot.GPS2.location.getY(DistanceUnit.INCH)));
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

            telemetry.addData("Time left", 117-runtime.seconds());
            //telemetry.addData("April tag range", myBot.camera.aprilTagRange);
            //telemetry.addData("April tag bearing", myBot.camera.aprilTagBearing);



            telemetry.update();
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
            if(gamepad1.x && Math.abs(targetAngle-shootAngle)<25)
                myBot.chassis.drive(0,0,(targetAngle-shootAngle)*.04);
            else
                myBot.chassis.driveFC(gamepad1.left_stick_y,-gamepad1.left_stick_x,gamepad1.right_stick_x,(myBot.GPS2.location.getHeading(AngleUnit.DEGREES)+FC_offset));

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
                myBot.shooter.shooterPower = myBot.shooter.shooterPower +.001;
                if(myBot.shooter.shooterPower > 1) myBot.shooter.shooterPower=1;
            }
            if (gamepad2.left_bumper)
            {
                myBot.shooter.shooterPower = myBot.shooter.shooterPower -.001;
                if(myBot.shooter.shooterPower < 0) myBot.shooter.shooterPower=0;
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
            if (gamepad1.back)
            {
                resetDebounce++;
                if(resetDebounce==5)
                {
                    myBot.chassis.drive(0, 0, 0);
                    sleep(250);
                    myBot.GPS2.resetGPS();
                    FC_offset = 0;
                }
            }
            else
                resetDebounce=0;
        }
    }
}
