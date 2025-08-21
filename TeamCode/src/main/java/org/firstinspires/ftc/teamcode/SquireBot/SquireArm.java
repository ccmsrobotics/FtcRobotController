package org.firstinspires.ftc.teamcode.SquireBot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


public class SquireArm {
    private DcMotor armLift, armExtend;
    public int armLiftLocation, armExtendLocation, armLiftTarget, armExtendTarget;
//Constructor.  This is called when a new SquireArm is created.
    public SquireArm(HardwareMap hm){
        armLift = hm.get(DcMotor.class, "arm_lift");
        armExtend = hm.get(DcMotor.class, "arm_extend");
        armExtend.setDirection(DcMotor.Direction.REVERSE);
        armExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLift.setTargetPosition(0);
        armExtend.setTargetPosition(0);
        armLiftTarget = 0;
        armExtendTarget=0;
        armExtend.setPower(0);
        armLift.setPower(0);
        armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        updateEncoders();
    }
    public void startMatchTeleArm(){
        armLift.setTargetPosition(armLift.getCurrentPosition()+15); //arm is unpowered after auton and may drag won
        armLift.setPower(1);
        armExtend.setPower(1);
    }

    public void startMatchAtonArm(){
        armLift.setPower(1);
        armExtend.setPower(1);
    }
    public void updateEncoders(){
        armLiftLocation=armLift.getCurrentPosition();
        armExtendLocation = armLift.getCurrentPosition();
    }
    public void resetArm(){
        armLift.setMode((DcMotor.RunMode.RUN_USING_ENCODER));
        armLift.setPower(-0.2);
    }
    public void resetEncoders(){
        armLift.setPower(0);
        armExtend.setPower(0);
        armLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLiftTarget = 0;
        armExtendTarget=0;
    }
    public void liftSetPower(double pwr){
        armLift.setPower(pwr);
    }
    public void extendSetPower(double pwr){
        armExtend.setPower(pwr);
    }

    public void setExtendTarget(int requestedExtendLocation){
        if(armLiftTarget > 600)
            if(requestedExtendLocation<3000 && requestedExtendLocation > -1) {
                armExtendTarget = requestedExtendLocation;
                armExtend.setTargetPosition(requestedExtendLocation);
            }
        else {
                if(requestedExtendLocation<1900 && requestedExtendLocation > -1) {
                    armExtendTarget = requestedExtendLocation;
                    armExtend.setTargetPosition(requestedExtendLocation);
                }
            }
    }
    public void setArmLiftTarget(int requestedLiftLocation){
        if(armExtendTarget>1900)
        {
            if (requestedLiftLocation <1800 && requestedLiftLocation>1200)
            {
                armLiftTarget = requestedLiftLocation;
                armLift.setTargetPosition(requestedLiftLocation);
            }
        }
        else{
            if (requestedLiftLocation <1800 && requestedLiftLocation>0)
            {
                armLiftTarget = requestedLiftLocation;
                armLift.setTargetPosition(requestedLiftLocation);
            }
        }

    }
    public void updateLocation(){
        armLiftLocation=armLift.getCurrentPosition();
        armExtendLocation=armExtend.getCurrentPosition();
    }
    public boolean armBusy(){
        if(armExtend.isBusy() || armLift.isBusy())
            return true;
        else
            return false;
           }

}
