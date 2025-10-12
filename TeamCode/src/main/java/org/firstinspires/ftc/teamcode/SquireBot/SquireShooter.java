package org.firstinspires.ftc.teamcode.SquireBot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

public class SquireShooter {
    public DcMotor intake, shooterLeft, shooterRight;
    private HardwareMap myHardwareMap;
    private Servo stopper;
    public float shooterPower, intakePower;

    private boolean debouncePress, debounceOpen;
    double debounceStart;

    public SquireShooter(HardwareMap hm) {
        myHardwareMap = hm;
        intake = myHardwareMap.get(DcMotor.class, "intake");
        shooterLeft = myHardwareMap.get(DcMotor.class, "shooter_left");
        shooterRight = myHardwareMap.get(DcMotor.class, "shooter_right");
        intake.setDirection(DcMotor.Direction.FORWARD);
        shooterLeft.setDirection(DcMotor.Direction.REVERSE);
        shooterRight.setDirection(DcMotor.Direction.FORWARD);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakePower = 1.0;
        shooterPower = 1.0;

        stopper = hm.get(Servo.class, "stopper");
        closeStopper();

    }

    public void intakeOn() {
        intake.setPower(intakePower);

    }

    public void intakeOff() {
        intake.setPower(0);

    }
    //This calculates shooter motor power based on range from april tag
    public void setShooterPower(float range){
        if( range <0)
        {range = 0;}
        else if (range >100)
        {range = 100;}

        shooterPower=0.5+0.5*(range/100);
    }

    public void enableShooter(){
        shooterLeft.setPower(shooterPower);
        shooterRight.setPower(shooterPower);
    }
    public void disableShooter(){
        shooterLeft.setPower(0);
        shooterRight.setPower(0);
    }
    public void openStopper(){
        stopper.setPosition(0.25);
    }
    public void closeStopper(){
        stopper.setPosition(0.5);
    }
    public void shooterState(boolean button, double currentTime){
        if(button == true)
        {
            if (debouncePress == false)
            {
                debouncePress = true;
                debounceOpen = false;
                debounceStartPress = currentTime;
            }
            else{
                if (currentTime - debounceStartPress) > 100)
            {

            }
            }



        }
        else
        if (debounceOpen == false)
        {
            debouncePress = true;
            debounceOpen = false;
            debounceStart = currentTime;


    }
}