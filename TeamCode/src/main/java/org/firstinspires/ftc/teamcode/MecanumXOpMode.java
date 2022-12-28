package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.SmartGamepad;

@TeleOp(name="MecanumX OpMode", group="Linear Opmode")

public class MecanumXOpMode extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private MecanumX drivetrain = null;
    private SmartGamepad smartGamepad1 = null;
    private SmartGamepad smartGamepad2 = null;

//    private DcMotor slideMotor = null;

    @Override
    public void runOpMode() throws InterruptedException {

        Utilities.getSharedUtility().initialize(this);
        smartGamepad1 = new SmartGamepad(gamepad1);
        smartGamepad2 = new SmartGamepad(gamepad2);

        drivetrain = new MecanumX();
//        slideMotor = hardwareMap.get(DcMotor .class, "slideMotor");

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            Utilities.getSharedUtility().updateAll();

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double y_move = -smartGamepad1.left_stick_y;  // Note: push32ing stick forward gives negative value
            double x_move = smartGamepad1.left_stick_x;
            double r_move = smartGamepad1.right_stick_x;
            boolean dryRun = smartGamepad1.b;
            boolean auto_correct = false;

            if (smartGamepad1.dpad_changed()) {
                drivetrain.saveReferencePosition();
            }
            if (smartGamepad1.right_stick_released()) {
                drivetrain.saveReferenceAngle();
            }

            if (smartGamepad1.dpad_up || smartGamepad1.dpad_down || smartGamepad1.dpad_left || smartGamepad1.dpad_right) {
                if (smartGamepad1.dpad_left) {
                    x_move = -1.0;
                }
                if (smartGamepad1.dpad_right) {
                    x_move = 1.0;
                }
                if (smartGamepad1.dpad_up) {
                    y_move = 1.0;
                }
                if (smartGamepad1.dpad_down) {
                    y_move = -1.0;
                }
                dryRun = false;
                auto_correct = true;
            }

            if (smartGamepad1.y) {
                y_move = 1.0;
                r_move = 1.0;
                dryRun = false;
                auto_correct = true;
            }
            if (smartGamepad1.x) {
                y_move = -1.0;
                r_move = -1.0;
                dryRun = false;
                auto_correct = true;
            }

//            if (smartGamepad1.y) {
//                slideMotor.setPower(0.6);
//            } else if (smartGamepad1.x) {
//                slideMotor.setPower(-0.6);
//            } else {
//                slideMotor.setPower(0.0);
//            }

            drivetrain.setMovement(x_move, y_move, r_move, dryRun, auto_correct);

        }
    }
}

