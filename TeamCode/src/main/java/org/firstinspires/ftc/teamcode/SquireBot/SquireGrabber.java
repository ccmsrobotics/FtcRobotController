package org.firstinspires.ftc.teamcode.SquireBot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SquireGrabber {
    private Servo grabber, rotator;

    public SquireGrabber(HardwareMap hm){
        grabber = hm.get(Servo.class, "grabber");
        rotator = hm.get(Servo.class, "rotator");
    }

}
