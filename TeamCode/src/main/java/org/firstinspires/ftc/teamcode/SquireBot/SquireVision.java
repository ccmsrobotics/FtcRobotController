package org.firstinspires.ftc.teamcode.SquireBot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

public class SquireVision {
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    public SquireVision(HardwareMap hm){
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hm.get(WebcamName.class, "Webcam 1"), aprilTag);
    }
    public int WhichPatternTag(){
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id ==21||detection.id  ==22|| detection.id==23) {
                return detection.id;
            }
        }   // end for() loop
        return 0;
    }
}
