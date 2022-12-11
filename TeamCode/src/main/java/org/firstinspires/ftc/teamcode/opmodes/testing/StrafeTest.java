package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Command;
import org.firstinspires.ftc.teamcode.robot.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain3DW;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(group = "drive")
public class StrafeTest extends LinearOpMode {
    public static double DISTANCE = 48; // inches
    public static double DIR_RIGHT = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        CrabRobot robot = new CrabRobot(this,true);
        Drivetrain3DW drivetrain = new Drivetrain3DW(robot);
        robot.registerSubsystem((Subsystem) drivetrain);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Trajectory trajRight = drivetrain.trajectoryBuilder(new Pose2d())
                .strafeRight(DISTANCE)
                .build();
        Trajectory trajLeft = drivetrain.trajectoryBuilder(new Pose2d())
                .strafeLeft(DISTANCE)
                .build();
        waitForStart();


        if (isStopRequested()) return;

        if (DIR_RIGHT==1) {
            robot.runCommand((Command) drivetrain.followTrajectory(trajRight));
        } else {
            robot.runCommand((Command) drivetrain.followTrajectory(trajLeft));
        }


    }
}
