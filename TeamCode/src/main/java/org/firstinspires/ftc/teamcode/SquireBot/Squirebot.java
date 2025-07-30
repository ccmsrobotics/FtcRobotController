package org.firstinspires.ftc.teamcode.SquireBot;
import com.qualcomm.robotcore.*;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
public class Squirebot {
    public SquireChassis drive;
    public HardwareMap hardwareMap;
    public SquireGPS GPS;
    public Telemetry telemetry;

    public Squirebot(HardwareMap hm, Telemetry T){
        hardwareMap=hm;
        telemetry = T;
        drive = new SquireChassis(hardwareMap);
        GPS = new SquireGPS(hardwareMap,telemetry);

    }
}
