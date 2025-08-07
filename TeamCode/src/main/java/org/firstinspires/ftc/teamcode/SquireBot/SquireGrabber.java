package org.firstinspires.ftc.teamcode.SquireBot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SquireGrabber {
    private Servo grabber, rotator;
    public final double WRISTUPPERBASKET = 0.55;


    public SquireGrabber(HardwareMap hm){
        grabber = hm.get(Servo.class, "grabber");
        rotator = hm.get(Servo.class, "rotator");


    }
    public void openGrabber(){
        grabber.setPosition(.40);
    }
    public void closeGrabber(){
        grabber.setPosition(0.02);
    }
    public void setWristPosition(double position){
        if (position >0 && position <1.0)
            rotator.setPosition(position);

    }

}
