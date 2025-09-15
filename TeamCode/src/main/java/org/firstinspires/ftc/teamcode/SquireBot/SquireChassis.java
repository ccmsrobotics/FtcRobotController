package org.firstinspires.ftc.teamcode.SquireBot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SquireChassis {
    public DcMotor leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive;
    private HardwareMap myHardwareMap;
    public double maxSpeed;
    private double leftFrontPower,  rightFrontPower, leftBackPower, rightBackPower;

//constructor.  This will be called when a new SquireChassis is created. 
    public SquireChassis(HardwareMap hm){
        myHardwareMap = hm;
        leftFrontDrive = myHardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = myHardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = myHardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = myHardwareMap.get(DcMotor.class, "right_back_drive");
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        maxSpeed=1.0;
    }
    public void drive(double drive, double strafe, double rotation){
        leftFrontPower = drive + strafe + rotation;
        rightFrontPower = drive - strafe - rotation;
        leftBackPower = drive - strafe + rotation;
        rightBackPower = drive + strafe - rotation;
        double max;
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));
        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        leftFrontPower *= maxSpeed;
        rightFrontPower *= maxSpeed;
        leftBackPower *= maxSpeed;
        rightBackPower *= maxSpeed;

        // Send calculated power to wheels
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }
    public void driveFC(double drive, double strafe, double rotation, double currentDirectionDegrees) {
        double currentYawRadians = currentDirectionDegrees * 3.1415 / 180;
        double rotStrafe = strafe * Math.cos(-currentYawRadians) - drive * Math.sin(-currentYawRadians);
        double rotDrive = strafe * Math.sin(-currentYawRadians) + drive * Math.cos(-currentYawRadians);
        rotStrafe = rotStrafe * 1.1;
        drive(rotDrive,rotStrafe,rotation);
    }

}
