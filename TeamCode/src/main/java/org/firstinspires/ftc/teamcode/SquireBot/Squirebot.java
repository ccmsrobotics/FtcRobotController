package org.firstinspires.ftc.teamcode.SquireBot;
import com.qualcomm.robotcore.*;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
public class Squirebot {
    public SquireChassis drive;
    public SquireArm arm;
    public SquireGrabber claw;
    public HardwareMap hardwareMap;
    public SquireGPS GPS;
    public Telemetry telemetry;
    public double FORWARD_GAIN  =  0.035  ;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    public double STRAFE_GAIN =  0.025 ;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
    public double TURN_GAIN   =  0.0175  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    public double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    public double MAX_AUTO_STRAFE= 0.5;   //  Clip the strafing speed to this max value (adjust for your robot)
    public double MAX_AUTO_TURN  = 0.4;  //  Clip the turn speed to this max value (adjust for your robot)
    public double MIN_SPEED_CONSTANT = 3;
    public double ANGLE_ERROR_MULT = 2.5;


    public Squirebot(HardwareMap hm, Telemetry T){
        hardwareMap=hm;
        telemetry = T;
        drive = new SquireChassis(hardwareMap);
        GPS = new SquireGPS(hardwareMap,telemetry);
        arm = new SquireArm(hardwareMap);
        claw = new SquireGrabber(hardwareMap);
        //notes
    }
    public void startAton(){

    }
    public void startTele(){
        arm.armLift.setPower(1);
        claw.rotator.setPosition(.5);
        claw.grabber.setPosition(0.02);
    }
    public goToSpot(double yTargetLoc, double xTargetLoc, double yawTarget, double allowedLocError ){
        double xError;
        double yError;
        double yawError;
        double  forward           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)
        double maxError = allowedLocError+1;
            while((maxError >allowedLocError)){//Missing "opModeIsActive() &&"
                GPS.UpdateGPS();
        yError = yTargetLoc-GPS.GPS.y;
        xError =xTargetLoc-GPS.GPS.x;
        yawError =yawErrorCalc(yawTarget,GPS.GPS.h);
        maxError =Math.max(Math.abs(xError),Math.abs(yError));
        maxError=Math.max(maxError, Math.abs(yawError/ANGLE_ERROR_MULT));//yaw error is scaled so 1" error is equivilent to ANGLE_ERROR_MULT (2.5 deg)
        double currentYawRadians = GPS.GPS.h*3.1415/180;
        double rotX = xError * Math.cos(-currentYawRadians) - yError * Math.sin(-currentYawRadians);
        double rotY = xError * Math.sin(-currentYawRadians) + yError * Math.cos(-currentYawRadians);
        forward  = Range.clip(rotY * FORWARD_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        turn   = Range.clip(yawError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
        strafe = Range.clip(rotX * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
        if(maxError<MIN_SPEED_CONSTANT)//limit min speed of robot.  This compensates for friction at low error levels
        {
            forward =forward*MIN_SPEED_CONSTANT/maxError;
            turn = turn*MIN_SPEED_CONSTANT/maxError;
            strafe = strafe*MIN_SPEED_CONSTANT/maxError;
        }
        drive.drive(-forward, strafe, -turn);//motors are reversed in config.  This should be fixed.
    }
    //stop robot at end of move
    drive.drive(0,0,0);
}


    public double yawErrorCalc(double yawTarget, double yawCurrent) {
        // Determine the heading current error
        double headingError = yawTarget - yawCurrent;
        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;
        return headingError;
    }
}
