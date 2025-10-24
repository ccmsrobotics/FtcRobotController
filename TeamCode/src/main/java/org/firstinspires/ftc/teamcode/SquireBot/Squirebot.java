package org.firstinspires.ftc.teamcode.SquireBot;
import com.qualcomm.robotcore.*;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Squirebot {
    public SquireChassis chassis;
    public HardwareMap hardwareMap;
    public SquirePinpoint GPS2;
    public SquireVision camera;
    public SquireShooter shooter;
    public Telemetry telemetry;
    private LinearOpMode opMode;
    public double FORWARD_GAIN = 0.075;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    public double STRAFE_GAIN = 0.075;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
    public double TURN_GAIN = 0.0175;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    public double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    public double MAX_AUTO_STRAFE = 0.5;   //  Clip the strafing speed to this max value (adjust for your robot)
    public double MAX_AUTO_TURN = 0.4;  //  Clip the turn speed to this max value (adjust for your robot)
    public double MIN_SPEED_CONSTANT = 5; //This is used in goToSpot to have a minimum speed returned by formula to account for friction at low power levels resulting in robot not moving
    public double ANGLE_ERROR_MULT = 2.5; //


    public Squirebot(LinearOpMode myOpmode, HardwareMap hm, Telemetry T) {
        opMode = myOpmode;
        hardwareMap = hm;
        telemetry = T;
        chassis = new SquireChassis(hardwareMap);
        GPS2=new SquirePinpoint(hardwareMap, telemetry);
        camera = new SquireVision(hardwareMap);
        shooter = new SquireShooter(hardwareMap);
        //notes
    }


    //public void startTele() {arm.startMatchTeleArm();}

    public void goToSpot(double yTargetLoc, double xTargetLoc, double yawTarget, double allowedLocError) {
        double xError;
        double yError;
        double yawError;
        double forward = 0;        // Desired forward power/speed (-1 to +1)
        double strafe = 0;        // Desired strafe power/speed (-1 to +1)
        double turn = 0;        // Desired turning power/speed (-1 to +1)
        double maxError = allowedLocError + 1;
        while ((maxError > allowedLocError)&& opMode.opModeIsActive()) {//Missing "opModeIsActive() &&"
            GPS2.UpdateGPS();
            yError = yTargetLoc - GPS2.location.getX(DistanceUnit.INCH);
            xError = xTargetLoc + GPS2.location.getY(DistanceUnit.INCH);
            yawError = yawErrorCalc(yawTarget, GPS2.location.getHeading(AngleUnit.DEGREES));
            maxError = Math.max(Math.abs(xError), Math.abs(yError));
            maxError = Math.max(maxError, Math.abs(yawError / ANGLE_ERROR_MULT));//yaw error is scaled so 1" error is equivilent to ANGLE_ERROR_MULT (2.5 deg)
            double currentYawRadians = GPS2.location.getHeading(AngleUnit.DEGREES) * 3.1415 / 180;
            double rotX = xError * Math.cos(-currentYawRadians) - yError * Math.sin(-currentYawRadians);
            double rotY = xError * Math.sin(-currentYawRadians) + yError * Math.cos(-currentYawRadians);
            forward = Range.clip(rotY * FORWARD_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            turn = Range.clip(yawError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            strafe = Range.clip(rotX * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
            if (maxError < MIN_SPEED_CONSTANT)//limit min speed of robot.  This compensates for friction at low error levels
            {
                forward = forward * MIN_SPEED_CONSTANT / maxError;
                turn = turn * MIN_SPEED_CONSTANT / maxError;
                strafe = strafe * MIN_SPEED_CONSTANT / maxError;
            }
            chassis.drive(forward, strafe, -turn);
            telemetry.addData("X coordinate",-GPS2.location.getY(DistanceUnit.INCH));
            telemetry.addData("Y coordinate", GPS2.location.getX(DistanceUnit.INCH));
            telemetry.addData("Heading angle", GPS2.location.getHeading(AngleUnit.DEGREES));
            telemetry.update();
        }
        //stop robot at end of move
        chassis.drive(0, 0, 0);
    }


    public double yawErrorCalc(double yawTarget, double yawCurrent) {
        // Determine the heading current error
        double headingError = yawTarget - yawCurrent;
        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;
        return headingError;
    }
    public void autonShooter(){
      shooter.shooterPower = 0.8;
      shooter.enableShooter();
      opMode.sleep(750);
      shooter.openStopper();
      opMode.sleep(750);
      shooter.intakeOn();
      opMode.sleep(750);
      shooter.intakeOff();
      shooter.disableShooter();
      shooter.closeStopper();
    }
}
