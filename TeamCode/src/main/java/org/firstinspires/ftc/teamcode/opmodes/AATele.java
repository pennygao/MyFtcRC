package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.KnockerCommand;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
import android.util.Log;

@TeleOp
public class AATele extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CrabRobot robot = new CrabRobot(this, true);
//RESETS
//        robot.scoringSystem.setServoPosition(0.6);

        waitForStart();

        while (!isStopRequested()) {

            boolean buttonA = gamepad2.a; // enter Align
            boolean buttonB = gamepad2.b; // exit Align
            boolean buttonX = gamepad2.x;
            boolean buttonY = gamepad2.y;
            boolean leftBumper = gamepad2.left_bumper;
            boolean rightBumper = gamepad2.right_bumper;
            boolean slowMode = gamepad1.left_bumper;
            boolean normieMode = gamepad1.right_bumper;
            float leftTrigger2 = gamepad2.left_trigger;
            float rightTrigger2 = gamepad2.right_trigger;
            double lTrigger1 = gamepad2.left_stick_x; //gamepad1.left_trigger;
            double rTrigger1 = gamepad2.right_stick_x;

            //telemetry.addData("mode:", robot.scoringSystem.slideMotor.getMode());
            //telemetry.addData("slide motor power: ", robot.scoringSystem.slideMotor.getPower());

            robot.update();
            Log.v("updatetarget", "robot.update() is done");

            //check the bottom of the code (A) for the deleted bit i commented out

//DRIVE
            robot.mecanumDrive.setDrivePower(new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x));
            if (slowMode) {
                robot.mecanumDrive.setPowerFactor(0.4);
            }
            if (normieMode) {
                robot.mecanumDrive.setPowerFactor(0.8);
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
                KnockerCommand knock = new KnockerCommand(robot, 0.4, 1.5);
                robot.runCommands(knock);
            }

/* SSKnocker thing?????
            if (gamepad1.left_bumper) {
                robot.scoringSystem.SSKnockerSetPosition(0.0);
            } else if (gamepad1.right_bumper) {
                robot.scoringSystem.SSKnockerSetPosition(0.45);
            }
*/
//RAISE SLIDE
            if (gamepad2.dpad_up) {
                robot.scoringSystem.adjustLift(1);
                telemetry.addLine("dpad up pressed");
            } else if (gamepad2.dpad_down) {
                robot.scoringSystem.adjustLift(-1);
                telemetry.addLine("dpad down pressed");
            } else if (buttonA) {
                //robot.scoringSystem.goToHt(18.69);
                robot.scoringSystem.dualMotorLift.goToLevel(1);
                telemetry.addLine("going up to level 1");
            } else if (buttonX) {
                //robot.scoringSystem.goToHt(31.5);
                robot.scoringSystem.dualMotorLift.goToLevel(2);
                telemetry.addLine("going up to level 2");
            } else if (buttonY) {
                //robot.scoringSystem.goToHt(44.69);
                robot.scoringSystem.dualMotorLift.goToLevel(3);
                telemetry.addLine("going up to level 3");
            } else if (buttonB) {
                //robot.scoringSystem.goToHt(4.42);
                robot.scoringSystem.goAllDown();
                telemetry.addLine("going all the way down");
            } else if (robot.scoringSystem.isLiftLevelReached()){
                robot.scoringSystem.adjustLift(0); //set to adjust lift mode, but don't turn motor
                Log.v("updatetarget","idling mode");
                //or set its mode to Move with encoder?
            }
//FIXME use game pad 1's triggers not joystick
            if(lTrigger1>0) {
                robot.scoringSystem.swingChainBar(-1);
            }
            else if (lTrigger1<0){
                robot.scoringSystem.swingChainBar(1);
            }
            if(rTrigger1>0) {
                robot.scoringSystem.adjustChainBar(-1);
            }
            else if (rTrigger1<0){
                robot.scoringSystem.adjustChainBar(1);
            }

            if (leftTrigger2 > 0.5) {
                robot.scoringSystem.closeClaw(); //0.5 + leftTrigger * 0.5);
            } else if (rightTrigger2 > 0.5) {
                robot.scoringSystem.openClaw(); // - rightTrigger * 0.5);
            }
            Log.v("updatetarget", "Opmode loop finished one iteration.");

        }
    }
}



