package org.firstinspires.ftc.teamcode.opmodes.testing;


import static org.firstinspires.ftc.teamcode.subsystems.Drivetrain3DW.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.subsystems.Drivetrain3DW.MAX_VEL;
import static org.firstinspires.ftc.teamcode.subsystems.Drivetrain3DW.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.subsystems.Drivetrain3DW.kA;
import static org.firstinspires.ftc.teamcode.subsystems.Drivetrain3DW.kStatic;
import static org.firstinspires.ftc.teamcode.subsystems.Drivetrain3DW.kV;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.commands.AutoCB;
import org.firstinspires.ftc.teamcode.commands.AutoClaw;
import org.firstinspires.ftc.teamcode.commands.AutoLift;
import org.firstinspires.ftc.teamcode.robot.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain3DW;
import org.firstinspires.ftc.teamcode.subsystems.DualMotorLift;

import java.util.Objects;

/*
 * This routine is designed to tune the open-loop feedforward coefficients. Although it may seem unnecessary,
 * tuning these coefficients is just as important as the positional parameters. Like the other
 * manual tuning routines, this op mode relies heavily upon the dashboard. To access the dashboard,
 * connect your computer to the RC's WiFi network. In your browser, navigate to
 * https://192.168.49.1:8080/dash if you're using the RC phone or https://192.168.43.1:8080/dash if
 * you are using the Control Hub. Once you've successfully connected, start the program, and your
 * robot will begin moving forward and backward according to a motion profile. Your job is to graph
 * the velocity errors over time and adjust the feedforward coefficients. Once you've found a
 * satisfactory set of gains, add them to the appropriate fields in the DriveConstants.java file.
 *
 * Pressing Y/Δ (Xbox/PS4) will pause the tuning process and enter driver override, allowing the
 * user to reset the position of the bot in the event that it drifts off the path.
 * Pressing B/O (Xbox/PS4) will cede control back to the tuning process.
 */
@Config
@Autonomous(group = "test")
public class ManualSlideTuner extends LinearOpMode {

    public static double H_DISTANCE = 32; // in
    public static double L_DISTANCE = 6;
    public static double ITERATIONS = 2;

    //private FtcDashboard dashboard = FtcDashboard.getInstance();

    enum Mode {
        DRIVER_MODE,
        TUNING_MODE
    }

    private Mode mode;

    @Override
    public void runOpMode() {
        CrabRobot robot = new CrabRobot(this,true);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        DualMotorLift lift =  new DualMotorLift(robot, telemetry, DualMotorLift.Mode.BOTH_MOTORS_PID);
        robot.registerSubsystem((Subsystem) lift);
        Drivetrain3DW drivetrain =  new Drivetrain3DW(robot);
        robot.registerSubsystem((Subsystem) drivetrain);

        mode = Mode.TUNING_MODE;

        NanoClock clock = NanoClock.system();
        AutoCB cbLeft = new AutoCB(robot, -1, 2);
        AutoCB cbDown = new AutoCB(robot, 0, 2);
        AutoClaw clawClose = new AutoClaw(robot, 0, 0.1);
        AutoClaw clawOpen = new AutoClaw(robot, 1, 0.05);

        telemetry.addLine("Ready!");
        telemetry.update();
        telemetry.clearAll();

        waitForStart();

        if (isStopRequested()) return;

        boolean movingForwards = true;
        double profileStart = clock.seconds();




        AutoLift liftUp = new AutoLift(robot, 5, H_DISTANCE);
        AutoLift liftDown = new AutoLift(robot, 5, L_DISTANCE);


        for (int i=0; i<ITERATIONS; i++) {
            AutoLift liftCmd = new AutoLift(robot, 5, 6-i);
            robot.runCommand(drivetrain.followTrajectorySequence(
                    drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                            //.splineTo(new Vector2d(HI_POLE_X-6, 20), Math.toRadians(-90)) // move forward
                            .back(44.5)
                            //.strafeLeft(2)
                            .addTemporalMarker(0.0, ()->robot.runCommands(new AutoLift(robot, 5, 30)))
                            .addTemporalMarker(0.5, ()->robot.runCommands(cbLeft))
                            .build()
            ));
            robot.runCommands(clawOpen);
            robot.runCommand(drivetrain.followTrajectorySequence(
                    drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                            .forward(44.5)
                            //.lineTo(new Vector2d())
                            .addTemporalMarker(1.0, ()->robot.runCommands(cbDown))
                            .addTemporalMarker(1.0, ()->robot.runCommands(liftCmd))
                            .build()
            ));
            robot.runCommands(clawClose);
            //Log.v("AUTODEBUG", (12 + 2*i - 1) + ": repick done");
        }

    }
}

