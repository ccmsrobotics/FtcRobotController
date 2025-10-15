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

    private boolean debounceActive;
    private double debounceStart, stateStart;
    public int currentState;

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
        currentState = 0;
        debounceActive=false;

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
        if(currentState ==0){
            //What should be always disabled?

            if(stateDebounce(button,currentTime,250)){
                currentState=1;
                stateStart=currentTime;
            }
        }
        else if(currentState==1){
            //what commands should be run

            if(stateDebounce(!button,currentTime,250)){
                currentState=0;
            }
            else if((currentTime-stateStart)> 2500){
                currentState=2;
                stateStart=currentTime;
            }
        }

        else if(currentState==2){
            //what commands should be run
            if(stateDebounce(!button,currentTime,250)){
                currentState=0;
            }
            else if((currentTime-stateStart)> 2500){
                currentState=3;
                stateStart=currentTime;
            }

        }
        else if(currentState==3){
            //what commands should be run
            if(stateDebounce(!button,currentTime,250)){
                currentState=0;
                intakeOff();
            }
        }
        else{currentState =0;}
    }

    //function used to debounce a button  If more then one instance needs to be used, should be turned into a class
    private boolean stateDebounce(boolean button, double currentTime, double debouncetime){
        if(button == true){
            if(debounceActive==true){
                if((currentTime-debounceStart) >debouncetime){
                    debounceActive=false;
                    return true;
                }
            }
            else
            {
                debounceStart=currentTime;
                debounceActive=true;
            }
        }
        else {
            debounceActive=false;
        }
        return false;
    }
}