/*
This sample FTC OpMode uses methods of the Datalogger class to specify and
collect robot data to be logged in a CSV file, ready for download and charting.

For instructions, see the tutorial at the FTC Wiki:
https://github.com/FIRST-Tech-Challenge/FtcRobotController/wiki/Datalogging


The Datalogger class is suitable for FTC OnBot Java (OBJ) programmers.
Its methods can be made available for FTC Blocks, by creating myBlocks in OBJ.

Android Studio programmers can see instructions in the Datalogger class notes.

Credit to @Windwoes (https://github.com/Windwoes).

*/


package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.SquireBot.SquireShooterEX;

@TeleOp(name = "Shooter Datalogging", group = "Datalogging")
public class ConceptDatalogger extends LinearOpMode
{
    Datalog datalog;
    VoltageSensor battery;
    SquireShooterEX shooter;
    public static final String TIMES_STARTED_KEY = "Times started";
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException
    {
        // Get devices from the hardwareMap.
        // If needed, change "Control Hub" to (e.g.) "Expansion Hub 1".
        battery = hardwareMap.voltageSensor.get("Control Hub");
        shooter = new SquireShooterEX(hardwareMap);
        // Initialize the datalog
        int timesStarted = (int) blackboard.getOrDefault(TIMES_STARTED_KEY, 0);
        blackboard.put(TIMES_STARTED_KEY, timesStarted + 1);
        int fileNumber = timesStarted +1;
        if(fileNumber <10)
        {
        datalog = new Datalog("datalog_0"+ fileNumber);
        }
        else {
            datalog = new Datalog("datalog_"+ fileNumber);
        }
            

        // You do not need to fill every field of the datalog
        // every time you call writeLine(); those fields will simply
        // contain the last value.
        datalog.opModeStatus.set("INIT");
        datalog.battery.set(battery.getVoltage());
        datalog.writeLine();
        shooter.shooterPower = 0.65;
        telemetry.setMsTransmissionInterval(50);
        while(!isStarted()) {
        if (gamepad1.right_bumper)
            {
                shooter.shooterPower = shooter.shooterPower +.0001;
                if(shooter.shooterPower > 1) shooter.shooterPower=1;
            }
            if (gamepad1.left_bumper)
            {
                shooter.shooterPower = shooter.shooterPower -.0001;
                if(shooter.shooterPower < 0) shooter.shooterPower=0;
            }
            telemetry.addLine("FileName datalog_0" + fileNumber);
            telemetry.addData("Shooter Power ", shooter.shooterPower);
            telemetry.update();     
            
        }
        
        waitForStart();
        runtime.reset();

        datalog.opModeStatus.set("RUNNING");

        for (int i = 0; opModeIsActive(); i++)
        {
            // Note that the order in which we set datalog fields
            // does *not* matter! The order is configured inside
            // the Datalog class constructor.

            shooter.shooterState(true, runtime.milliseconds());
            datalog.loopCounter.set(i);
            datalog.battery.set(battery.getVoltage());
            datalog.State.set(shooter.currentState);
            datalog.leftShooter.set(shooter.shooterLeft.getVelocity());
            datalog.rightShooter.set(shooter.shooterRight.getVelocity());
            datalog.intake.set(shooter.intake.getVelocity());
            datalog.leftShooterpwr.set(shooter.shooterLeft.getPower());
            datalog.rightShooterpwr.set(shooter.shooterRight.getPower());
            datalog.intakepwr.set(shooter.intake.getPower());
            datalog.leftCurrent.set(shooter.shooterLeft.getCurrent(CurrentUnit.AMPS));
            datalog.rightCurrent.set(shooter.shooterRight.getCurrent(CurrentUnit.AMPS));
            datalog.intakeCurrent.set(shooter.intake.getCurrent(CurrentUnit.AMPS));



            // The logged timestamp is taken when writeLine() is called.
            datalog.writeLine();

            // Datalog fields are stored as text only; do not format here.
            telemetry.addLine();
            telemetry.addData("OpMode Status", datalog.opModeStatus);
            telemetry.addData("Loop Counter", datalog.loopCounter);
            telemetry.addData("Battery", datalog.battery);

            telemetry.update();

            //sleep(20);
        }

        /*
         * The datalog is automatically closed and flushed to disk after 
         * the OpMode ends - no need to do that manually :')
         */
    }

    /*
     * This class encapsulates all the fields that will go into the datalog.
     */
    public static class Datalog
    {
        // The underlying datalogger object - it cares only about an array of loggable fields
        private final Datalogger datalogger;

        // These are all of the fields that we want in the datalog.
        // Note that order here is NOT important. The order is important in the setFields() call below
        public Datalogger.GenericField opModeStatus = new Datalogger.GenericField("OpModeStatus");
        public Datalogger.GenericField loopCounter  = new Datalogger.GenericField("Loop Counter");
        public Datalogger.GenericField State          = new Datalogger.GenericField("State");
        public Datalogger.GenericField leftShooter        = new Datalogger.GenericField("Left Velocity");
        public Datalogger.GenericField rightShooter         = new Datalogger.GenericField("Right Velocity");
        public Datalogger.GenericField intake         = new Datalogger.GenericField("Intake Velocity");
        public Datalogger.GenericField leftShooterpwr        = new Datalogger.GenericField("Left Power");
        public Datalogger.GenericField rightShooterpwr         = new Datalogger.GenericField("Right Power");
        public Datalogger.GenericField intakepwr         = new Datalogger.GenericField("Intake Power");
        public Datalogger.GenericField leftCurrent        = new Datalogger.GenericField("Left Current");
        public Datalogger.GenericField rightCurrent         = new Datalogger.GenericField("Right Current");
        public Datalogger.GenericField intakeCurrent         = new Datalogger.GenericField("Intake Current");
        public Datalogger.GenericField battery      = new Datalogger.GenericField("Battery");

        public Datalog(String name)
        {
            // Build the underlying datalog object
            datalogger = new Datalogger.Builder()

                    // Pass through the filename
                    .setFilename(name)

                    // Request an automatic timestamp field
                    .setAutoTimestamp(Datalogger.AutoTimestamp.DECIMAL_SECONDS)

                    // Tell it about the fields we care to log.
                    // Note that order *IS* important here! The order in which we list
                    // the fields is the order in which they will appear in the log.
                    .setFields(
                            opModeStatus,
                            loopCounter,
                            State,
                            leftShooter,
                            rightShooter,
                            intake,
                            leftShooterpwr,
                            rightShooterpwr,
                            intakepwr,
                            leftCurrent,
                            rightCurrent,
                            intakeCurrent,
                            battery
                    )
                    .build();
        }

        // Tell the datalogger to gather the values of the fields
        // and write a new line in the log.
        public void writeLine()
        {
            datalogger.writeLine();
        }
    }
}
