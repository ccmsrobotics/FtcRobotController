package org.firstinspires.ftc.teamcode.SquireBot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SquireShooterEX {
    public DcMotorEx intake, shooterLeft, shooterRight;
    private HardwareMap myHardwareMap;
    public double shooterPower, intakePower;

    private boolean debounceActive;
    private double debounceStart, stateStart;
    public int currentState;

    public SquireShooterEX(HardwareMap hm) {
        myHardwareMap = hm;
        intake = myHardwareMap.get(DcMotorEx.class, "intake");
        shooterLeft = myHardwareMap.get(DcMotorEx.class, "shooter_left");
        shooterRight = myHardwareMap.get(DcMotorEx.class, "shooter_right");
        intake.setDirection(DcMotorEx.Direction.FORWARD);
        shooterLeft.setDirection(DcMotorEx.Direction.FORWARD);
        shooterRight.setDirection(DcMotorEx.Direction.REVERSE);
        intake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooterLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooterRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooterRight.setVelocityPIDFCoefficients(10,1,0,12.6);
        shooterLeft.setVelocityPIDFCoefficients(10,1,0,12.6);
        intakePower = 1.0;
        shooterPower = .45;
        currentState = 0;
        debounceActive=false;

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
            intakeBackwards();

            if(stateDebounce(!button,currentTime,250)){
                currentState=0;
                intakeOff();
            }
            else if((currentTime-stateStart)> 250){
                currentState=2;
                stateStart=currentTime;
                intakeOff();
            }
        }

        else if(currentState==2){
            //what commands should be run
            enableShooter();
            if(stateDebounce(!button,currentTime,250)){
                currentState=0;
                disableShooter();
            }
            else if((currentTime-stateStart)> 1000){
                currentState=3;
                stateStart=currentTime;
            }

        }
        else if(currentState==3){
            //what commands should be run
            enableShooter();
            intakePower=0.6;
            intakeOn();
            if(stateDebounce(!button,currentTime,250)){
                currentState=0;
                disableShooter();
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
    public void intakeBackwards(){
        intake.setPower(-0.5);
    }
}