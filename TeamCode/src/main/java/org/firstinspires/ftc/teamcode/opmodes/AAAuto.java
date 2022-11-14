package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.autoLift;
import org.firstinspires.ftc.teamcode.commands.autoLiftInch;
import org.firstinspires.ftc.teamcode.commands.autoLiftDown;
import org.firstinspires.ftc.teamcode.robot.Command;
import org.firstinspires.ftc.teamcode.robot.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

@Autonomous
public class AAAuto extends LinearOpMode {
    public static double HI_POLE_X = 26.5;
    public static double HI_POLE_SIDE = 13;
    public static double HI_POLE_FWD = 6;
    public static double HI_POLE_HEADING = Math.toRadians(40); // degree
    public static double POLE_HT = 32.0;

    @Override
    public void runOpMode() throws InterruptedException {
        CrabRobot robot = new CrabRobot(this);
        Drivetrain drivetrain = new Drivetrain(robot);
        robot.registerSubsystem((Subsystem) drivetrain);

        // Update following parameters
        double intakePower = 0.6;
        double outtakePower = 0;

        // general variable
        double driveTime;

        // Commands



        //Servo init code here
        robot.outtake.setRollerPower(0.5);
        waitForStart();
        if (isStopRequested()) return;

        // hold preload
        robot.runCommand(robot.outtake.rollerIntake(intakePower, 0.8));

        // Move to high pole
        robot.runCommand(drivetrain.followTrajectorySequence(
                drivetrain.trajectorySequenceBuilder(new Pose2d())
                        .forward(HI_POLE_X) // move forward
                        .strafeRight(HI_POLE_SIDE)
                        .build()
        ));

        // Raise lift
        autoLift liftUp = new autoLift(robot, 3, POLE_HT);
        robot.runCommands(liftUp);

        // Forward a little
        robot.runCommand(drivetrain.followTrajectorySequence(
                drivetrain.trajectorySequenceBuilder(new Pose2d())
                        .forward(HI_POLE_FWD) // move forward
                        .build()
        ));

        // Release cone
        robot.runCommand(robot.outtake.rollerIntake(outtakePower, 0.5));

        // Back a little
        robot.runCommand(drivetrain.followTrajectorySequence(
                drivetrain.trajectorySequenceBuilder(new Pose2d())
                        .back(HI_POLE_FWD) // move forward
                        .build()
        ));

        // retract lift
        autoLift liftDown = new autoLift(robot, 0, 0);
        robot.runCommands(liftDown);

        // park
        robot.runCommand(drivetrain.followTrajectorySequence(
                drivetrain.trajectorySequenceBuilder(new Pose2d())
                        .strafeLeft(12) // move side ways
                        .build()
        ));

    }
}

