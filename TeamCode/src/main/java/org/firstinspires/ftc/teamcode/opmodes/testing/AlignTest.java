package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.SmartGamepad;
import org.firstinspires.ftc.teamcode.commands.TeleAlign;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;

import android.util.Log;

@TeleOp
public class AlignTest extends LinearOpMode {
    public static double ALIGN_PWR = 0.2;

    @Override
    public void runOpMode() throws InterruptedException {
        CrabRobot robot = new CrabRobot(this, true);
        robot.addGamepads(gamepad1,gamepad2);
        SmartGamepad smartGamepad1 = robot.smartGamepad1;
        SmartGamepad smartGamepad2 = robot.smartGamepad2;

        TeleAlign autoLfCmd = new TeleAlign(robot, robot.mecanumDrive, ALIGN_PWR, true, telemetry);
        TeleAlign autoRtCmd = new TeleAlign(robot, robot.mecanumDrive, ALIGN_PWR, false, telemetry);


        waitForStart();

        while (!isStopRequested()) {

            boolean slowMode = (gamepad1.left_trigger > 0.5);
            boolean normieMode = (gamepad1.right_trigger > 0.5);


            robot.update();
            Log.v("DualMotorSlide-updatetarget", "robot.update() is done");

            //check the bottom of the code (A) for the deleted bit i commented out

//DRIVE
            robot.mecanumDrive.setDrivePower(new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x));
            if (slowMode) {
                robot.mecanumDrive.setPowerFactor(0.3);
            }
            if (normieMode) {
                robot.mecanumDrive.setPowerFactor(0.6);
            }


            // Chain bar and auto-align
            if(smartGamepad1.dpad_left) {
                robot.scoringSystem.swingChainBar(1); // left for tele
                robot.runCommand(autoLfCmd);
            }
            else if (smartGamepad1.dpad_right){
                robot.scoringSystem.swingChainBar(-1);
                robot.runCommand(autoRtCmd);
            }

            // Claw
            if (smartGamepad1.left_bumper) {
                robot.scoringSystem.claw.closeClaw();
                robot.scoringSystem.dualMotorLift.goToLevel(2);//0.5 + leftTrigger * 0.5);
            } else if (smartGamepad1.right_bumper) {
                robot.scoringSystem.claw.openClaw(); // - rightTrigger * 0.5);
            }

            // Lift levels
            if (smartGamepad1.a_pressed()) {
                //robot.scoringSystem.goToHt(18.69);
                robot.scoringSystem.dualMotorLift.goToLevel(1);
                telemetry.addLine("going up to level 1");
                Log.v("PIDLift: gamepad", "a");
            } else if (smartGamepad1.x_pressed()) {
                //robot.scoringSystem.goToHt(31.5);
                robot.scoringSystem.dualMotorLift.goToLevel(2);
                telemetry.addLine("going up to level 2");
                Log.v("PIDLift: gamepad", "x");
            } else if (smartGamepad1.y_pressed()) {
                //robot.scoringSystem.goToHt(44.69);
                robot.scoringSystem.dualMotorLift.goToLevel(3);
                telemetry.addLine("going up to level 3");
                Log.v("PIDLift: gamepad", "y");
            } else if (smartGamepad1.b_pressed()) {
                //robot.scoringSystem.goToHt(4.42);
                robot.scoringSystem.goAllDown();
                telemetry.addLine("going all the way down");
                Log.v("PIDLift: gamepad", "b");
            } else if (robot.scoringSystem.isLiftLevelReached()){
                robot.scoringSystem.dualMotorLift.stopMotor(); //set to adjust lift mode, but don't turn motor
                Log.v("updatetarget","idling mode");
                //or set its mode to Move with encoder?
            }


            if(gamepad1.dpad_up){
                telemetry.addData("left distance:", robot.robotdistancesensor.dsR);
                telemetry.addData("Right distance:", robot.robotdistancesensor.dsL);
            }


            Log.v("updatetarget", "Opmode loop finished one iteration.");

        }
    }
}

