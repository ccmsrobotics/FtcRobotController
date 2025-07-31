package org.firstinspires.ftc.teamcode.SquireBot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class SquireArm {
    public DcMotor armLift, armExtend;
    public int

    public SquireArm(HardwareMap hm){
        armLift = hm.get(DcMotor.class, "arm_lift");
        armExtend = hm.get(DcMotor.class, "arm_extend");
        armExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armExtend.setDirection(DcMotor.Direction.REVERSE);
        armLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armLift.setTargetPosition(0);
        armLift.setPower(0);
        armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
    public void setArmExtend(int targetArmLocation){

    }
    public void setArmLift(int targetLiftLocation){

    }
    public int armExtendLocation(){
        return armExtend.getCurrentPosition();
    }
    public int armLiftLocation(){
        return armLift.getCurrentPosition();
    }
}
