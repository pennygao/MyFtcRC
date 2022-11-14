package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.commands.autoLift;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

@TeleOp
public class AATele extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CrabRobot robot = new CrabRobot(this);

        int slidecountup = 0;
        int slidecountdown = 0;
        boolean isApressed = false;
        boolean isXpressed = false;
        boolean isYpressed = false;
        boolean inTransfer = false;

        boolean slideMotor_useEncoder = true;



//RESETS
//        robot.outtake.setServoPosition(0.6);
        if (!slideMotor_useEncoder) {
            robot.outtake.slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        waitForStart();

        while (!isStopRequested()) {

            boolean buttonA = gamepad2.a; // enter Align
            boolean buttonB = gamepad2.b; // exit Align
            boolean buttonX = gamepad2.x;
            boolean buttonY = gamepad2.y;
            boolean leftBumper = gamepad2.left_bumper;
            boolean rightBumper = gamepad2.right_bumper;
            boolean slowMode = gamepad1.a;
            boolean normieMode = gamepad1.b;
            float leftTrigger = gamepad2.left_trigger;
            float rightTrigger = gamepad2.right_trigger;
            //robot.outtake.goToLevel(0);
            telemetry.addData("slide level init: ", robot.outtake.getLevel());
//            telemetry.addData("dumpServo Position:",robot.outtake.getDumpPosition());
            telemetry.update();

            robot.update();

            //check the bottom of the code (A) for the deleted bit i commented out

//DRIVE
            robot.mecanumDrive.setDrivePower(new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x));

            if (slowMode) {
                robot.mecanumDrive.setPowerFactor(0.2);
            }
            if (normieMode) {
                robot.mecanumDrive.setPowerFactor(0.8);
            }

//INTAKE


//RAISE SLIDE
            if (slideMotor_useEncoder) { // use encoder
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

            } else {

                if (gamepad2.dpad_up) {
                    robot.outtake.slideMotor.setPower(0.5);
                } else if (gamepad2.dpad_down) {
                    robot.outtake.slideMotor.setPower(-0.5);
                } else {
                    robot.outtake.slideMotor.setPower(0.0);
                }
            }



            // Roller intake
            /*
            if (leftTrigger!= 0.0 || rightTrigger != 0.0) {
                double rollerPower = 0.5 + leftTrigger * 0.5 - rightTrigger * 0.5;
                robot.outtake.setRollerPower(rollerPower);
                telemetry.addData("Roller power: ", rollerPower);
            }
            */

            if (leftBumper) {
                robot.outtake.setRollerPower(0.5);
            } else if (leftTrigger != 0) {
                robot.outtake.setRollerPower(0.5 + leftTrigger * 0.5);
            } else if (rightTrigger != 0.0) {
                robot.outtake.setRollerPower(0.5 - rightTrigger * 0.5);
            }


//DUMP
            /*
            if(leftBumper) {
                int level = robot.outtake.getLevel();
                double servoPosition=0.6;
                switch (level){
                    case 1: servoPosition=0.25; //0.47 before fine-tune; hits the outer part of the tray
                    break;
                    case 2: servoPosition= 0.25;
                    break;
                    case 3: servoPosition= 0.25;
                    break;
                }
                robot.outtake.setServoPosition(servoPosition);
                telemetry.addLine("dumping  ");
            }



            if (rightBumper) {
                robot.outtake.setServoPosition(0.6);
                telemetry.addLine("resetting dumper");
            }

             */


//DUCK SPINNER

        }
    }
}


