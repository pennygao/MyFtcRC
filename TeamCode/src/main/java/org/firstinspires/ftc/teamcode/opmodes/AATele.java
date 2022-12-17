package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.commands.KnockerCommand;
import org.firstinspires.ftc.teamcode.commands.autoLift;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

@TeleOp
public class AATele extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CrabRobot robot = new CrabRobot(this, true);







//RESETS
//        robot.outtake.setServoPosition(0.6);

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
            float leftTrigger = gamepad2.left_trigger;
            float rightTrigger = gamepad2.right_trigger;
            //telemetry.addData("mode:", robot.outtake.slideMotor.getMode());
            //telemetry.addData("slide motor power: ", robot.outtake.slideMotor.getPower());

            robot.update();

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
                robot.outtake.SSKnockerSetPosition(0.0);
            } else if (gamepad1.right_bumper) {
                robot.outtake.SSKnockerSetPosition(0.45);
            }
*/
//RAISE SLIDE
            if (gamepad2.dpad_up) {
                //robot.outtake.setSlideMotorMode(false);
                //robot.outtake.slideMotor.setPower(1.0);
                robot.outtake.goUp1Inch();
                telemetry.addLine("dpad up pressed");
            } else if (gamepad2.dpad_down) {
                //robot.outtake.setSlideMotorMode(false);
                robot.outtake.goDown1Inch();
                //robot.outtake.slideMotor.setPower(-0.7);
                telemetry.addLine("dpad down pressed");
            } else if (buttonA) {
                //robot.outtake.setSlideMotorMode(true);
                robot.outtake.goToHt(18.69);
                telemetry.addLine("going up to level 1");
            } else if (buttonX) {
                //robot.outtake.setSlideMotorMode(true);
                robot.outtake.goToHt(31.5);
                telemetry.addLine("going up to level 2");
            } else if (buttonY) {
                //robot.outtake.setSlideMotorMode(true);
                robot.outtake.goToHt(44.69);
                telemetry.addLine("going up to level 3");
            } else if (buttonB) {
                //robot.outtake.setSlideMotorMode(true);
                robot.outtake.goToHt(4.42);
                telemetry.addLine("going all the way down");
            }


            if (leftTrigger > 0.5) {
                robot.outtake.setRollerPower(0.5); //0.5 + leftTrigger * 0.5);
            } else if (rightTrigger > 0.5) {
                robot.outtake.setRollerPower(0.1); // - rightTrigger * 0.5);
            }

        }
    }
}



