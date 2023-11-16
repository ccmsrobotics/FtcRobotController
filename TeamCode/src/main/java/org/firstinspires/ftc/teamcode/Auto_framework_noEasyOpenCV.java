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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;


import java.util.ArrayList;

/*
 * FTC Team 18975 autonomous code
 */
@Autonomous
public class Auto_framework_noEasyOpenCV extends LinearOpMode {
    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorSimple lift = null;
    private DcMotorSimple intake = null;
    private Servo dumpTruck = null;

    // Used to manage the video source.
    private AprilTagProcessor aprilTag;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    AprilTagDetection tagOfInterest = null;
    static final double FEET_PER_METER = 3.28084;
    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    // UNITS ARE METERS
    double tagsize = 0.166;
    int numFramesWithoutDetection = 0;
    int tagMissingFrames = 0;
    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;
    int ID_TAG_OF_INTEREST = 18;
    private helmetLocationPipeline helmetPipeline;
    boolean readyToDeliver=false;
    boolean pixel_delivered=false;
    double drive = 0;
    double turn = 0;
    double strafe = 0;

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
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);



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
            sleep(1500);
            lift.setPower(0);
            myPosition = helmetPipeline.getAnalysis();
            sleep(200);
            myPosition = helmetPipeline.getAnalysis();
            telemetry.addData("Analysis", helmetPipeline.position);
            telemetry.update();
            //webcam.setPipeline(aprilTagDetectionPipeline);
            sleep(100);
            if (helmetPipeline.position == Auto_framework_noEasyOpenCV.helmetLocationPipeline.helmetPosition.LEFT) {
                //set april tag ID needed for the backdrop unload  Blue left is 1, Red left is 4
                ID_TAG_OF_INTEREST = 4;

                //shift 12 inches left

                moveRobot(0, -0.5, 0);
                //runtime.reset();
                sleep(450);
                moveRobot(-1, 0, 0);
                //runtime.reset();
                sleep(550);
                //stop robot
                stopRobot();
                sleep(400);
                //unload pixel

                intake.setPower(0.4);
                sleep(1400);
                intake.setPower(0);
                //move to center
                moveRobot(-0.5, 0, 0);
                sleep(500);
                stopRobot();
                sleep(200);
                //rotate to face wall
                moveRobot(0,0,-0.7);
                sleep(500);
                stopRobot();
                sleep(200);
                //move forward 48 inches
                moveRobot(-1,0,0);
                sleep(1500);
                stopRobot();
                sleep(400);
                moveRobot(-0.5,0,0);
                sleep(800);
                stopRobot();
                moveRobot(0,0.2,0);
                sleep(5000);
                stopRobot();
                sleep(50000);
                //transistion to april tag unload
            } else if (helmetPipeline.position == Auto_framework_noEasyOpenCV.helmetLocationPipeline.helmetPosition.CENTER) {
                //set april tag ID needed for the backdrop unload  Blue center is 2, Red left is 5
                ID_TAG_OF_INTEREST = 2;
                moveRobot(-1, 0, 0);
                //runtime.reset();
                sleep(775);
                //stop robot
                stopRobot();
                sleep(200);
                //unload pixel

                intake.setPower(0.4);
                sleep(700);
                intake.setPower(0);
                //move to center
                //moveRobot(-0.5, 0, 0);
                //sleep(400);
                //stopRobot();
                sleep(200);
                //rotate to face wall
                moveRobot(0,0,-0.7);
                sleep(425);
                stopRobot();
                sleep(200);
                //move forward 48 inches
                moveRobot(-1,0,0);
                sleep(1500);
                stopRobot();
                sleep(400);
                moveRobot(-0.5,0,0);
                sleep(800);
                stopRobot();
                moveRobot(0,0.2,0);
                sleep(5000);
                stopRobot();
                sleep(50000);

                //transistion to april tag unload
            } else {
                //set april tag ID needed for the backdrop unload Blue right is 1, Red right is 4
                ID_TAG_OF_INTEREST = 1;
                moveRobot(0,-0.5,0);
                sleep(200);
                moveRobot(-1, 0, 0);
                sleep(450);
                //stop robot
                stopRobot();
                sleep(200);
                moveRobot(0,0,.65);
                sleep(500);
                moveRobot(.5,0,0);
                sleep(350);
                stopRobot();
                //unload pixel

                intake.setPower(0.35);
                sleep(700);
                intake.setPower(0);
                moveRobot(-.5,0,0);
                sleep(500);
                moveRobot(0,1,0);
                sleep(725);
                moveRobot(0,0,-.7);
                sleep(1100);
                stopRobot();

                //move to center
                moveRobot(-0.5, 0, 0);
                sleep(500);
                stopRobot();
                sleep(200);
                //rotate to face wall
                //moveRobot(0,0,0.5);
                //sleep(200);
                ///stopRobot();
                //sleep(200);
                //move forward 48 inches
                moveRobot(-1,0,0);
                sleep(1500);
                stopRobot();
                sleep(400);
                moveRobot(-0.5,0,0);
                sleep(400);
                stopRobot();
                moveRobot(0,0.2,0);
                sleep(5000);
                stopRobot();
                sleep(50000);

                //transistion to april tag unload
            }
            webcam.setPipeline(aprilTagDetectionPipeline);
            lift.setPower(.75);
            sleep(200);
            lift.setPower(0);
            sleep(100);
            //This loop uses apriltag to drop to backdrop
            while (!readyToDeliver) {
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
                            double rangeError = (tagOfInterest.pose.z*36 - 18);
                            double headingError = tagOfInterest.pose.x*36;
                            double yawError = rot.firstAngle;
                            telemetry.addLine(String.format("\nDetected tag ID=%d", tagOfInterest.id));
                            telemetry.addLine(String.format("Translation X: %.2f inches", tagOfInterest.pose.z * 36));
                            telemetry.addLine(String.format("Translation Y: %.2f inches", tagOfInterest.pose.x * 36));
                            telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", rot.firstAngle));
                            //
                            if (rangeError < 2 && headingError < 2 && yawError < 5) {
                                readyToDeliver = true;
                                stopRobot();
                                break; //end the While loop
                            }
                            drive = Range.clip(-rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                            turn = Range.clip(-headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                            moveRobot(drive, turn, strafe);
                        } else {
                            tagMissingFrames++;
                            if (tagMissingFrames > 3) {
                                stopRobot();
                            }
                        }
                    }
                    telemetry.update();
                }
                sleep(50);
            }

            //deposit

            dumpTruck.setPosition(-1);
            sleep(1000);
            //jiggle the unload
            dumpTruck.setPosition(-.8);
            sleep(500);
            dumpTruck.setPosition(-1);
            sleep(500);
            dumpTruck.setPosition(0);
            //move to backstage
            moveRobot(-.2,0,0);
            sleep(400);
            moveRobot(0,.5,0);
            sleep(400);
            moveRobot(.25,0,0);
            sleep(400);
            stopRobot();
            lift.setPower(0);
            intake.setPower(0);
            //Loop until end of match
            while(true){

            }

        }
    }


    public class helmetLocationPipeline implements VisionProcessor {


        public enum helmetPosition {
            NONE,
            LEFT,
            CENTER,
            RIGHT
        }

        public Rect rectLeft = new Rect(0, 140, 135, 95);
        public Rect rectMiddle = new Rect(320, 120, 135, 95);
        public Rect rectRight = new Rect(643, 160, 135, 95);
        helmetPosition selection = helmetPosition.NONE;

        Mat submat = new Mat();
        Mat hsvMat = new Mat();

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);

            double satRectLeft = getAvgSaturation(hsvMat, rectLeft);
            double satRectMiddle = getAvgSaturation(hsvMat, rectMiddle);
            double satRectRight = getAvgSaturation(hsvMat, rectRight);

            if ((satRectLeft > satRectMiddle) && (satRectLeft > satRectRight)) {
                return helmetPosition.LEFT;
            } else if ((satRectMiddle > satRectLeft) && (satRectMiddle > satRectRight)) {
                return helmetPosition.CENTER;
            }
            return helmetPosition.RIGHT;
        }

        protected double getAvgSaturation(Mat input, Rect rect) {
            submat = input.submat(rect);
            Scalar color = Core.mean(submat);
            return color.val[1];
        }

        private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
            int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
            int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
            int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
            int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

            return new android.graphics.Rect(left, top, right, bottom);
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
            Paint selectedPaint = new Paint();
            selectedPaint.setColor(Color.RED);
            selectedPaint.setStyle(Paint.Style.STROKE);
            selectedPaint.setStrokeWidth(scaleCanvasDensity * 4);

            Paint nonSelectedPaint = new Paint(selectedPaint);
            nonSelectedPaint.setColor(Color.GREEN);

            android.graphics.Rect drawRectangleLeft = makeGraphicsRect(rectLeft, scaleBmpPxToCanvasPx);
            android.graphics.Rect drawRectangleMiddle = makeGraphicsRect(rectMiddle, scaleBmpPxToCanvasPx);
            android.graphics.Rect drawRectangleRight = makeGraphicsRect(rectRight, scaleBmpPxToCanvasPx);

            selection = (helmetPosition) userContext;
            switch (selection) {
                case LEFT:
                    canvas.drawRect(drawRectangleLeft, selectedPaint);
                    canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
                    canvas.drawRect(drawRectangleRight, nonSelectedPaint);
                    break;
                case CENTER:
                    canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
                    canvas.drawRect(drawRectangleMiddle, selectedPaint);
                    canvas.drawRect(drawRectangleRight, nonSelectedPaint);
                    break;
                case RIGHT:
                    canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
                    canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
                    canvas.drawRect(drawRectangleRight, selectedPaint);
                    break;
                case NONE:
                    canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
                    canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
                    canvas.drawRect(drawRectangleRight, nonSelectedPaint);
                    break;
            }
        }

        public helmetPosition getSelection() {
            return selection;
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
}

