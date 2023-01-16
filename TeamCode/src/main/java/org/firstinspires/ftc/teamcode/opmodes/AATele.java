package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.SmartGamepad;
import org.firstinspires.ftc.teamcode.commands.KnockerCommand;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
import android.util.Log;

@TeleOp
public class AATele extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CrabRobot robot = new CrabRobot(this, true);
        robot.addGamepads(gamepad1,gamepad2);
        SmartGamepad smartGamepad1 = robot.smartGamepad1;
        SmartGamepad smartGamepad2 = robot.smartGamepad2;
//RESETS
        //robot.scoringSystem.goAllDown();

        waitForStart();

        while (!isStopRequested()) {

            boolean slowMode = gamepad1.left_bumper;
            boolean normieMode = gamepad1.right_bumper;

            //telemetry.addData("mode:", robot.scoringSystem.slideMotor.getMode());
            //telemetry.addData("slide motor power: ", robot.scoringSystem.slideMotor.getPower());

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

//Color sensor
            if (gamepad1.a) {
                telemetry.addData("RRed: ", robot.robotcolorsensor.csR[0]);
                telemetry.addData("RGreen: ", robot.robotcolorsensor.csR[1]);
                telemetry.addData("RBlue: ", robot.robotcolorsensor.csR[2]);
                telemetry.addData("RonLine:", robot.robotcolorsensor.csRonLine());
                telemetry.addLine("Gamepad1 A pressed");
                telemetry.update();
            } else if (gamepad1.b) {
                telemetry.addData("LRed: ", robot.robotcolorsensor.csL[0]);
                telemetry.addData("LGreen: ", robot.robotcolorsensor.csL[1]);
                telemetry.addData("LBlue: ", robot.robotcolorsensor.csL[2]);
                telemetry.addData("LonLine:", robot.robotcolorsensor.csLonLine());
                telemetry.addLine("Gamepad1 B pressed");
                telemetry.update();
            }

//SS knocker
            if (gamepad2.right_bumper){
                robot.scoringSystem.SSKnockerSetPosition(0.1);
            }
            else if (gamepad2.left_bumper){
                robot.scoringSystem.SSKnockerSetPosition(0.55);
            }
//RAISE SLIDE
            if (smartGamepad2.dpad_up) {
                robot.scoringSystem.adjustLift(1, true);
                telemetry.addLine("dpad up pressed");
                Log.v("PIDLift: gamepad", "dpad up");
            } else if (smartGamepad2.dpad_down) {
                robot.scoringSystem.adjustLift(-1, true);
                telemetry.addLine("dpad down pressed");
                Log.v("PIDLift: gamepad", "dpad down");
            } else if (smartGamepad2.dpad_right) {
                robot.scoringSystem.adjustLift(1, false);
                telemetry.addLine("dpad right pressed");
                Log.v("PIDLift: gamepad", "dpad right");
            } else if (smartGamepad2.dpad_left) {
                robot.scoringSystem.adjustLift(-1, false);
                telemetry.addLine("dpad left pressed");
                Log.v("PIDLift: gamepad", "dpad left");
            }
            else if (smartGamepad2.a_pressed()) {
                //robot.scoringSystem.goToHt(18.69);
                robot.scoringSystem.dualMotorLift.goToLevel(1);
                telemetry.addLine("going up to level 1");
                Log.v("PIDLift: gamepad", "a");
            } else if (smartGamepad2.x_pressed()) {
                //robot.scoringSystem.goToHt(31.5);
                robot.scoringSystem.dualMotorLift.goToLevel(2);
                telemetry.addLine("going up to level 2");
                Log.v("PIDLift: gamepad", "x");
            } else if (smartGamepad2.y_pressed()) {
                //robot.scoringSystem.goToHt(44.69);
                robot.scoringSystem.dualMotorLift.goToLevel(3);
                telemetry.addLine("going up to level 3");
                Log.v("PIDLift: gamepad", "y");
            } else if (smartGamepad2.b_pressed()) {
                //robot.scoringSystem.goToHt(4.42);
                robot.scoringSystem.goAllDown();
                telemetry.addLine("going all the way down");
                Log.v("PIDLift: gamepad", "b");
            } else if (robot.scoringSystem.isLiftLevelReached()){
                robot.scoringSystem.dualMotorLift.stopMotor(); //set to adjust lift mode, but don't turn motor
                Log.v("updatetarget","idling mode");
                //or set its mode to Move with encoder?
            }
            if(smartGamepad1.left_trigger_pressed()) {
                robot.scoringSystem.swingChainBar(1); // left for tele
            }
            else if (smartGamepad1.right_trigger_pressed()){
                robot.scoringSystem.swingChainBar(-1);
            }
            if(smartGamepad2.right_stick_x>0) {
                robot.scoringSystem.adjustChainBar(-1);
            }
            else if (smartGamepad2.right_stick_x<0){
                robot.scoringSystem.adjustChainBar(1);
            }

            if (smartGamepad2.left_trigger > 0.5) {
                robot.scoringSystem.claw.closeClaw(); //0.5 + leftTrigger * 0.5);
            } else if (smartGamepad2.right_trigger > 0.5) {
                robot.scoringSystem.claw.openClaw(); // - rightTrigger * 0.5);
            }
            Log.v("updatetarget", "Opmode loop finished one iteration.");

        }
    }
}



