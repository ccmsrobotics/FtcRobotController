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
        armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        updateEncoders();
    }
    public void startMatchTeleArm(){
        armLift.setTargetPosition(armLift.getCurrentPosition()+15); //arm is unpowered after auton and may drag won
        armLift.setPower(1);
        armExtend.setPower(1);
    }

    public void startMatchAtonArm(){

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

    public void setExtendTarget(int targetArmLocation){
        int armLiftLocation = armLift.getCurrentPosition();
        int armExtendLocation = armExtend.getCurrentPosition();
        if (armLiftLocation < 600) {
            if (armExtendLocation < 400) {
                if (gamepad2.a)
                    armExtendPower = 1;
                else
                    armExtendPower = 0;
            } else if (armExtendLocation < 1900) {
                if (gamepad2.a)
                    armExtendPower = 1;
                else if (gamepad2.b)
                    armExtendPower = -1;
                else
                    armExtendPower = 0;
            } else {
                if (gamepad2.b)
                    armExtendPower = -1;
                else
                    armExtendPower = 0;
            }
        } else if (armLiftLocation > 600) {
            if (armExtendLocation < 400) {
                if (gamepad2.a)
                    armExtendPower = 1;
                else
                    armExtendPower = 0;
            } else if (armExtendLocation < 2750) {
                if (gamepad2.a)
                    armExtendPower = 1;
                else if (gamepad2.b)
                    armExtendPower = -1;
                else
                    armExtendPower = 0;
            } else if (armExtendLocation < 3000) {
                if (gamepad2.a)
                    armExtendPower = 0.5;
                else if (gamepad2.b)
                    armExtendPower = -0.5;
                else
                    armExtendPower = 0;
            } else {
                if (gamepad2.b)
                    armExtendPower = -1;
                else
                    armExtendPower = 0;
            }
        }

    }
    public void setArmLiftTarget(int targetLiftLocation){
        if(targetLiftLocation > 0 && targetLiftLocation < 1800){
            if(armExtendLocation > 1900) {
                if(targetLiftLocation > 1200) {
                    armLift.setTargetPosition(targetLiftLocation);
                }
            }
            else {
                armLift.setTargetPosition(targetLiftLocation);
            }

        }

    }

}
