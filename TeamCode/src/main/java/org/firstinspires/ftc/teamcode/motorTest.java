package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SquireBot.Squirebot;


@TeleOp(name = "Shoot Test", group = "Class")
//@Disabled
public class motorTest extends LinearOpMode {

    // Declare variables used by the class

    @Override
    public void runOpMode() {

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Shoot Test", "Initialized");
        telemetry.update();
        DcMotor ShootMotor;
        ShootMotor = hardwareMap.get(DcMotor.class, "shoot");
        //Start of TeleOp
        waitForStart();

        while (opModeIsActive()) {

            telemetry.addLine("Shoot Test");
            telemetry.addData("power", gamepad1.left_stick_y);

            telemetry.update();
            ShootMotor.setPower(gamepad1.left_stick_y);
            }
        }

    }
