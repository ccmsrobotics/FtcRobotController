package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SquireBot.BobShooterEX;
import org.firstinspires.ftc.teamcode.SquireBot.Squirebot;


@TeleOp(name = "bob", group = "Class")
//@Disabled
public class TeleOp_Bob extends LinearOpMode {

    // Declare variables used by the class
    BobShooterEX bob;
    private ElapsedTime runtime = new ElapsedTime();
    private double headingError = 0;
    private double targetAngle = 0;
    private String alliance;
    int resetDebounce = 0;
    int kickDebounce = 0;
    double driveAngle, shootAngle;
    double calcXOffset, calcYOffset;

    @Override
    public void runOpMode() {
        bob = new BobShooterEX(hardwareMap);


        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Start of TeleOp
        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            telemetry.addData("Shooter State", bob.currentState);
            telemetry.addData("Shooter Power", bob.shooterPower);
            telemetry.addData("Time left", 117 - runtime.seconds());
            telemetry.update();

            if (gamepad1.right_trigger > 0.3) {
                bob.intakePower = gamepad1.right_trigger;
                bob.intakeOn();
            } else if (gamepad2.dpad_left) {
                bob.intakeBackwards();
            } else if (bob.currentState > 2) {
            } else {
                bob.intakeOff();
            }
            bob.shooterState(gamepad1.a || gamepad2.a, runtime.milliseconds());

            if (gamepad2.x) {
                bob.shooterPower = 0.697;
            }
            if (gamepad2.a) {
                bob.shooterPower = 0.718;
            }
            if (gamepad2.y) {
                bob.shooterPower = 0.697;
            }
            if (gamepad2.b) {
                bob.shooterPower = 0.718;
            }
            if (gamepad2.right_bumper) {
                bob.shooterPower = bob.shooterPower + .00001;
                if (bob.shooterPower > 1) bob.shooterPower = 1;
            }
            if (gamepad2.left_bumper) {
                bob.shooterPower = bob.shooterPower - .00001;
                if (bob.shooterPower < 0) bob.shooterPower = 0;
            }

            if (gamepad2.dpad_up) {
                bob.enableShooter();
            }
            if (gamepad2.right_stick_y > .2 || gamepad2.right_stick_y < 0.2) {
                bob.intakePower = gamepad2.right_stick_y;
            }


            if (gamepad2.dpad_down)
                bob.disableShooter();
            if (gamepad2.dpad_left) {
                bob.intakeBackwards();
            }

        }

    }
}
