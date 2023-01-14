package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.robot.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain3DW;
import org.firstinspires.ftc.teamcode.util.Encoder;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "test")
public class LocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CrabRobot robot = new CrabRobot(this,false);
        //Drivetrain3DW drivetrain = new Drivetrain3DW(robot);
        Drivetrain drivetrain = new Drivetrain(robot);
        robot.registerSubsystem((Subsystem) drivetrain);
        Encoder leftEncoder = new Encoder(robot.getHardwareMap().get(DcMotorEx.class, "leftEncoder"));
        Encoder rightEncoder = new Encoder(robot.getHardwareMap().get(DcMotorEx.class, "slideMotorR"));
        Encoder frontEncoder = new Encoder(robot.getHardwareMap().get(DcMotorEx.class, "frontEncoder"));

        waitForStart();

        while (!isStopRequested()) {
            robot.update();
            drivetrain.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * 0.2,
                            -gamepad1.left_stick_x * 0.2,
                            -gamepad1.right_stick_x * 0.2
                    ));

            Pose2d poseEstimate = drivetrain.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("leftEncoder:", leftEncoder.getCurrentPosition());
            telemetry.addData("rightEncoder:", rightEncoder.getCurrentPosition());
            telemetry.addData("frontEncoder:", frontEncoder.getCurrentPosition());
            telemetry.update();
        }
    }
}
