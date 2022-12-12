package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;

@TeleOp
public class AATele2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CrabRobot robot = new CrabRobot(this, false);


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
            telemetry.addData("mode:", robot.outtake.slideMotor.getMode());
            telemetry.addData("slide motor power: ", robot.outtake.slideMotor.getPower());
//            telemetry.addData("dumpServo Position:",robot.outtake.getDumpPosition());
            telemetry.update();

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

//INTAKE


//RAISE SLIDE
            /*
                if (buttonA) {
                    if (!isApressed) {
                        robot.outtake.goUp();
                        isApressed = true;
                        telemetry.addData("going up. level: ", robot.outtake.targetPosition);
                    }
                } else {
                    isApressed = false;
                }

                if (buttonB) {
                    robot.outtake.goalldown();
                    ///telemetry.addData("going down to: ", robot.outtake.targetPosition);
                }

                if (buttonX) {
                    robot.outtake.goUp1Inch();
                }
                if (buttonY) {
                    robot.outtake.goDown1Inch();
                }
          */


            if (gamepad2.dpad_up) {
                //robot.outtake.setSlideMotorMode(false);
                //robot.outtake.slideMotor.setPower(1.0);
                robot.outtake.slideMotor.setPower(0.4);
                telemetry.addLine("going up slowly");
            } else if (gamepad2.dpad_down) {
                //robot.outtake.setSlideMotorMode(false);
                robot.outtake.slideMotor.setPower(-0.4);
                //robot.outtake.slideMotor.setPower(-0.7);
                telemetry.addLine("going down slowly");
            } else if (buttonX) {
                //robot.outtake.setSlideMotorMode(true);
                robot.outtake.slideMotor.setPower(-0.7);
                telemetry.addLine("going down");
            } else if (buttonY) {
                //robot.outtake.setSlideMotorMode(true);
                robot.outtake.slideMotor.setPower(0.7);
                telemetry.addLine("going up");
            }
            else{
                robot.outtake.slideMotor.setPower(0);
            }


                telemetry.update();

                if (leftBumper) {
                    robot.outtake.setRollerPower(0.5);
                } else if (leftTrigger != 0) {
                    robot.outtake.setRollerPower(0.5 + leftTrigger * 0.5);
                } else if (rightTrigger != 0.0) {
                    robot.outtake.setRollerPower(0.5 - rightTrigger * 0.5);
                }

        }
    }
}



