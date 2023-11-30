/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

/*
 * FTC Team 18975 autonomous code
 */
@Autonomous
public class Auto_Blue_Unload extends LinearOpMode {
    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorSimple lift = null;
    private DcMotorSimple intake = null;
    private Servo dumpTruck = null;
    OpenCvWebcam webcam;
    helmetLocationPipeline helmetPipeline;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    AprilTagDetection tagOfInterest = null;
    static final double FEET_PER_METER = 3.28084;

    private double  headingError  = 0;
    private double  targetHeading = 0;
    private double  turnSpeed     = 0;
    static final double     HEADING_THRESHOLD       = 1.0 ;
    // Lens intrinsics
    // UNITS ARE PIXELS
/*    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
  */
    //C720 webcam at 800x448
    double fx = 872.04;
    double fy = 872.04;
    double cx = 308.741;
    double cy = 178.678;


    // UNITS ARE METERS
    double tagsize = 0.0508;
    int numFramesWithoutDetection = 0;
    int tagMissingFrames = 0;
    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;
    int ID_TAG_OF_INTEREST = 18;
    boolean readyToDeliver=false;
    boolean pixel_delivered=false;
    double drive = 0;
    double turn = 0;
    double strafe = 0;
    IMU imu;

    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    @Override
    public void runOpMode() {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        lift = hardwareMap.get(DcMotorSimple.class, "lift");
        intake = hardwareMap.get(DcMotorSimple.class, "intake");
        dumpTruck = hardwareMap.get(Servo.class, "dump_truck");
        imu = hardwareMap.get(IMU.class, "imu");
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));


        //Initialize the Helmet location pipeline
        helmetPipeline = new helmetLocationPipeline();
        helmetLocationPipeline.helmetPosition myPosition;
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        //Initialize camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(helmetPipeline);
        webcam.setMillisecondsPermissionTimeout(5000);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        while (!isStarted()) {
            myPosition = helmetPipeline.getAnalysis();
        }
        waitForStart();
        while (opModeIsActive()) {
            //Find location of team element
            lift.setPower(-0.35);
            sleep(500);
            lift.setPower(0);
            dumpTruck.setPosition(1);
            sleep(1500);
            lift.setPower(.35);
            sleep(1000);
            lift.setPower(0);
            myPosition = helmetPipeline.getAnalysis();
            sleep(200);
            myPosition = helmetPipeline.getAnalysis();
            telemetry.addData("Analysis", helmetPipeline.position);
            telemetry.update();
            //webcam.setPipeline(aprilTagDetectionPipeline);
            sleep(100);
            if (helmetPipeline.position == Auto_Blue_Unload.helmetLocationPipeline.helmetPosition.LEFT) {
                //set april tag ID needed for the backdrop unload  Blue left is 1, Red left is 4
                ID_TAG_OF_INTEREST = 1;
                //shift 12 inches left
                moveRobot(-0.3,0,0);
                sleep(300);
                moveRobot(0, 0.35, 0);
                //runtime.reset();
                sleep(800);
                moveRobot(-.6, 0, 0);
                //runtime.reset();
                sleep(1600);
                //stop robot
                stopRobot();
                sleep(400);
                //unload pixel
                intake.setPower(0.3);
                sleep(1400);
                intake.setPower(0);
                //move to center
                moveRobot(-0.5, 0, 0);
                sleep(300);
                stopRobot();
                sleep(200);
                //rotate to face wall
                //moveRobot(0,0,-0.7);
                moveRobot(0,0,0.4);
                sleep(1000);
                stopRobot();
                sleep(500);
                //transistion to april tag unload
            } else if (helmetPipeline.position == Auto_Blue_Unload.helmetLocationPipeline.helmetPosition.CENTER) {
                //set april tag ID needed for the backdrop unload  Blue center is 2, Red left is 5
                ID_TAG_OF_INTEREST = 2;
                moveRobot(-0.8, 0, 0);
                //runtime.reset();
                sleep(1500);
                //stop robot
                stopRobot();
                sleep(200);
                //unload pixel

                intake.setPower(0.3);
                sleep(700);
                intake.setPower(0);
                //move to center
                moveRobot(-0.5, 0, 0);
                sleep(250);
                //stopRobot();
                sleep(200);
                //rotate to face wall
                moveRobot(0,0,0.7);
                sleep(640);
                stopRobot();
                sleep(200);
                //move forward 48 inches
                moveRobot(-.8,0,0);
                sleep(350);
                stopRobot();
                //sleep(400);
                //moveRobot(-0.5,0,0);
                sleep(500);
                stopRobot();
                moveRobot(0,-0.75,0);
                sleep(1150);
                stopRobot();
                sleep(50);

                //transistion to april tag unload
            } else {
                //set april tag ID needed for the backdrop unload Blue right is 1, Red right is 4
                ID_TAG_OF_INTEREST = 3;

                sleep(50000);

                //transistion to april tag unload
            }
            webcam.setPipeline(aprilTagDetectionPipeline);
            lift.setPower(-.75);
            sleep(800);
            lift.setPower(0);
            sleep(100);
            //This loop uses apriltag to drop to backdrop
            while (!readyToDeliver && opModeIsActive()) {
                ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();
               // If we don't see any tags
                if (detections != null) {
                    if (detections.size() == 0) {
                        numFramesWithoutDetection++;
                        telemetry.addLine("No tag Found");
                        // If we haven't seen a tag for a few frames, lower the decimation
                        // so we can hopefully pick one up if we're e.g. far back
                        if (numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) {
                            aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
                            stopRobot();
                        }
                    }
                    // We do see tags!
                    else {
                        boolean tagFound = false;
                        telemetry.addLine("Tag(s) Found");
                        //go through the detections and see if the desired tag is available
                        for (AprilTagDetection tag : detections) {
                            if (tag.id == ID_TAG_OF_INTEREST) {
                                tagOfInterest = tag;
                                tagFound = true;
                                //telemetry.addLine("Tag is correct",tag.id);
                                break;
                            }
                        }
                        numFramesWithoutDetection = 0;
                        if (tagFound) {
                            tagMissingFrames = 0;
                            if (tagOfInterest.pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS) {
                                aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
                            }
                            Orientation rot = Orientation.getOrientation(tagOfInterest.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);
                            /*
                            double rangeError = (tagOfInterest.pose.z*39.37 - 24);
                            double headingError = tagOfInterest.pose.x*39.37;
                            double yawError = rot.firstAngle;
                            */
                            //new error code
                            //https://github.com/FIRST-Tech-Challenge/ftcdocs/blob/main/docs/source/apriltag/vision_portal/apriltag_intro/apriltag-intro.rst
                            double  rangeError      = (Math.hypot(tagOfInterest.pose.x,tagOfInterest.pose.z)*39.37 - 18);
                            double  headingError    = AngleUnit.DEGREES.fromUnit(AngleUnit.RADIANS,Math.atan2(-tagOfInterest.pose.x, tagOfInterest.pose.z));
                            double  yawError        = -rot.firstAngle;
                            //end new error code
                            telemetry.addLine(String.format("\nDetected tag ID=%d", tagOfInterest.id));
                            telemetry.addLine(String.format("Translation X: %.2f inches", tagOfInterest.pose.z * 39.37));
                            telemetry.addLine(String.format("Translation Y: %.2f inches", tagOfInterest.pose.x * 39.37));
                            telemetry.addLine(String.format("Translation Z: %.2f inches", tagOfInterest.pose.y * 39.37));
                            telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", rot.firstAngle));
                            telemetry.addLine(String.format("Rotation Second: %.2f degrees", rot.secondAngle));
                            telemetry.addLine(String.format("Rotation Third: %.2f degrees", rot.thirdAngle));
                            telemetry.addLine(String.format("Range Error %.2f", rangeError));
                            telemetry.addLine(String.format("Heading Error: %.2f degrees", headingError));
                            telemetry.addLine(String.format("yawError: %.2f degrees", yawError));


                            //
                            if (rangeError < 2 && headingError < 1 && yawError < 2) {
                                readyToDeliver = true;
                                stopRobot();
                                break; //end the While loop
                            }
                            /* Tested (but wrong formulas)
                            drive = Range.clip(-rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                            strafe = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                            turn = Range.clip(yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                            */
                            //Original formulas
                            drive  = Range.clip(-rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                            turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                            strafe = Range.clip(yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                            moveRobot(drive, strafe, turn);
                        } else {
                            tagMissingFrames++;
                            if (tagMissingFrames > 3) {
                                stopRobot();
                            }
                        }
                    }
                    telemetry.update();
                }
                sleep(20);
            }

            //deposit
            moveRobot(-0.2, 0, 0);
            //runtime.reset();
            sleep(950);
            //stop robot
            stopRobot();
            dumpTruck.setPosition(-1);
            sleep(1000);
            //jiggle the unload
            dumpTruck.setPosition(-.8);
            sleep(500);
            dumpTruck.setPosition(-1);
            sleep(500);
            dumpTruck.setPosition(1);
            //move to backstage
            moveRobot(.2,0,0);
            sleep(400);
            moveRobot(0,-.5,0);
            sleep(1400);
            moveRobot(-.25,0,0);
            sleep(1500);
            stopRobot();
            lift.setPower(0);
            intake.setPower(0);
            sleep(50000);

            //Loop until end of match

        }
    }


    public static class helmetLocationPipeline extends OpenCvPipeline {
        /*
         * An enum to define the Helmet position
         */
        public enum helmetPosition {
            LEFT,
            CENTER,
            RIGHT
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(0, 140);
        static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(320, 120);
        static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(643, 160);
        static final int REGION_WIDTH = 135;
        static final int REGION_HEIGHT = 95;

        /*
         * Points which actually define the sample region rectangles, derived from above values
         *
         * Example of how points A and B work to define a rectangle
         *
         *   ------------------------------------
         *   | (0,0) Point A                    |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                  Point B (70,50) |
         *   ------------------------------------
         *
         */
        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region2_pointA = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x,
                REGION2_TOPLEFT_ANCHOR_POINT.y);
        Point region2_pointB = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region3_pointA = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x,
                REGION3_TOPLEFT_ANCHOR_POINT.y);
        Point region3_pointB = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb, region2_Cb, region3_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1, avg2, avg3;


        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile helmetPosition position = helmetPosition.LEFT;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            //Third variable controls color  0 is brightness, 1 is red, 2 is blue
            Core.extractChannel(YCrCb, Cb, 2);
        }

        @Override
        public void init(Mat firstFrame) {
            /*
             * We need to call this in order to make sure the 'Cb'
             * object is initialized, so that the submats we make
             * will still be linked to it on subsequent frames. (If
             * the object were to only be initialized in processFrame,
             * then the submats would become delinked because the backing
             * buffer would be re-allocated the first time a real frame
             * was crunched)
             */
            inputToCb(firstFrame);

            /*
             * Submats are a persistent reference to a region of the parent
             * buffer. Any changes to the child affect the parent, and the
             * reverse also holds true.
             */
            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
            region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));
            region3_Cb = Cb.submat(new Rect(region3_pointA, region3_pointB));
        }

        @Override
        public Mat processFrame(Mat input) {
            /*
             * Overview of what we're doing:
             *
             * We first convert to YCrCb color space, from RGB color space.
             * Why do we do this? Well, in the RGB color space, chroma and
             * luma are intertwined. In YCrCb, chroma and luma are separated.
             * YCrCb is a 3-channel color space, just like RGB. YCrCb's 3 channels
             * are Y, the luma channel (which essentially just a B&W image), the
             * Cr channel, which records the difference from red, and the Cb channel,
             * which records the difference from blue. Because chroma and luma are
             * not related in YCrCb, vision code written to look for certain values
             * in the Cr/Cb channels will not be severely affected by differing
             * light intensity, since that difference would most likely just be
             * reflected in the Y channel.
             *
             * After we've converted to YCrCb, we extract just the 2nd channel, the
             * Cb channel. We do this because stones are bright yellow and contrast
             * STRONGLY on the Cb channel against everything else, including SkyStones
             * (because SkyStones have a black label).
             *
             * We then take the average pixel value of 3 different regions on that Cb
             * channel, one positioned over each stone. The brightest of the 3 regions
             * is where we assume the SkyStone to be, since the normal stones show up
             * extremely darkly.
             *
             * We also draw rectangles on the screen showing where the sample regions
             * are, as well as drawing a solid rectangle over top the sample region
             * we believe is on top of the SkyStone.
             *
             * In order for this whole process to work correctly, each sample region
             * should be positioned in the center of each of the first 3 stones, and
             * be small enough such that only the stone is sampled, and not any of the
             * surroundings.
             */

            /*
             * Get the Cb channel of the input frame after conversion to YCrCb
             */
            inputToCb(input);

            /*
             * Compute the average pixel value of each submat region. We're
             * taking the average of a single channel buffer, so the value
             * we need is at index 0. We could have also taken the average
             * pixel value of the 3-channel image, and referenced the value
             * at index 2 here.
             */
            avg1 = (int) Core.mean(region1_Cb).val[0];
            avg2 = (int) Core.mean(region2_Cb).val[0];
            avg3 = (int) Core.mean(region3_Cb).val[0];

            /*
             * Draw a rectangle showing sample region 1 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            /*
             * Draw a rectangle showing sample region 2 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            /*
             * Draw a rectangle showing sample region 3 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines


            /*
             * Find the max of the 3 averages
             */
            int maxOneTwo = Math.max(avg1, avg2);
            int max = Math.max(maxOneTwo, avg3);

            /*
             * Now that we found the max, we actually need to go and
             * figure out which sample region that value was from
             */
            if (max == avg1) // Was it from region 1?
            {
                position = helmetPosition.LEFT; // Record our analysis

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region1_pointA, // First point which defines the rectangle
                        region1_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            } else if (max == avg2) // Was it from region 2?
            {
                position = helmetPosition.CENTER; // Record our analysis

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region2_pointA, // First point which defines the rectangle
                        region2_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            } else if (max == avg3) // Was it from region 3?
            {
                position = helmetPosition.RIGHT; // Record our analysis

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region3_pointA, // First point which defines the rectangle
                        region3_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }

            /*
             * Render the 'input' buffer to the viewport. But note this is not
             * simply rendering the raw camera feed, because we called functions
             * to add some annotations to this buffer earlier up.
             */
            return input;
        }

        /*
         * Call this from the OpMode thread to obtain the latest analysis
         */
        public helmetPosition getAnalysis() {
            return position;
        }
    }


    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower = x - y - yaw;
        double rightFrontPower = x + y + yaw;
        double leftBackPower = x + y - yaw;
        double rightBackPower = x - y + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }
    public void stopRobot() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
    public void turnToHeading(double maxTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, SPEED_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, 0, turnSpeed);
            telemetry.addLine(String.format("Turn speed: %.2f",turnSpeed));
            telemetry.addLine(String.format("Rotation: %.2f",getHeading()));
            telemetry.update();
        }

        // Stop all motion;
        moveRobot(0, 0,0);
    }
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Determine the heading current error
        headingError = targetHeading - getHeading();

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);

        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", rot.firstAngle));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", rot.secondAngle));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", rot.thirdAngle));
    }
}

