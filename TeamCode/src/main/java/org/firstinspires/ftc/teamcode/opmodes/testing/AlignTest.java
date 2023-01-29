package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.SmartGamepad;
import org.firstinspires.ftc.teamcode.commands.KnockerCommand;
import org.firstinspires.ftc.teamcode.commands.DriveRaiseDumpFold;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
import org.firstinspires.ftc.teamcode.subsystems.RobotDistanceSensor;
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

        DriveRaiseDumpFold autoLfCmd = new DriveRaiseDumpFold(robot, robot.mecanumDrive, ALIGN_PWR, true, telemetry);
        DriveRaiseDumpFold autoRtCmd = new DriveRaiseDumpFold(robot, robot.mecanumDrive, ALIGN_PWR, false, telemetry);


        waitForStart();

        while (!isStopRequested()) {

            boolean slowMode = gamepad1.left_bumper;
            boolean normieMode = gamepad1.right_bumper;


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



            if(smartGamepad1.dpad_left) {
                robot.scoringSystem.swingChainBar(1); // left for tele
                robot.runCommand(autoLfCmd);
            }
            else if (smartGamepad1.dpad_right){
                robot.scoringSystem.swingChainBar(-1);
                robot.runCommand(autoRtCmd);
            }

            if (smartGamepad1.left_bumper) {
                robot.scoringSystem.claw.closeClaw();
                robot.scoringSystem.dualMotorLift.goToLevel(2);//0.5 + leftTrigger * 0.5);
            } else if (smartGamepad1.right_bumper) {
                robot.scoringSystem.claw.openClaw(); // - rightTrigger * 0.5);
            }

/*
            if(gamepad1.dpad_left){
                telemetry.addData("left distance:", robot.robotdistancesensor.dsL);
            }
            if(gamepad1.dpad_right){
                telemetry.addData("Right distance:", robot.robotdistancesensor.dsR);
            }

 */
            Log.v("updatetarget", "Opmode loop finished one iteration.");

        }
    }
}

