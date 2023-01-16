package org.firstinspires.ftc.teamcode.opmodes;


import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.AutoCB;
import org.firstinspires.ftc.teamcode.commands.AutoClaw;
import org.firstinspires.ftc.teamcode.commands.AutoLift;
import org.firstinspires.ftc.teamcode.commands.DriveTillIntake;
import org.firstinspires.ftc.teamcode.commands.KnockerCommand;
import org.firstinspires.ftc.teamcode.robot.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.objectDetector;

@Config
@Autonomous
public class AutoLeft extends LinearOpMode {
    public static double HI_POLE_X = 57;
    public static double HI_POLE_Y = 4;
    public static double HI_POLE_SIDE = 14.5;
    public static double HI_POLE_FWD = 5.5;
    public static double HI_POLE_HEADING = Math.toRadians(40); // degree
    public static double POLE_HT = 43.69;

    @Override
    public void runOpMode() throws InterruptedException {
        CrabRobot robot = new CrabRobot(this,true);
        //Drivetrain3DW drivetrain = new Drivetrain3DW(robot);
        Drivetrain drivetrain = new Drivetrain(robot);
        robot.registerSubsystem((Subsystem) drivetrain);
        objectDetector od = new objectDetector(robot, telemetry);
        robot.registerSubsystem((Subsystem)od);

        int elementPos = 4;

        // Commands



        //Servo init code here
        robot.scoringSystem.SSKnockerSetPosition(0.6);
        robot.scoringSystem.chainBar.lower();
        //robot.scoringSystem.setClawPos(0.5);
        od.init();

        KnockerCommand knock = new KnockerCommand(robot, 0.05, 0.5);

        AutoCB cbLeft = new AutoCB(robot, 1, 2); // auto left is -1
        AutoCB cbDown = new AutoCB(robot, 0, 2); // down is 0
        AutoClaw clawClose = new AutoClaw(robot, 0, 1);
        AutoClaw clawOpen = new AutoClaw(robot, 1, 0.5);

        Trajectory traj1 = drivetrain.trajectoryBuilder(new Pose2d())
                //.splineTo(new Vector2d(HI_POLE_X, 0), 0) // move forward
                .lineTo(new Vector2d(HI_POLE_X, 0)) // move forward
                .addTemporalMarker(0.0, ()->robot.runCommands(clawClose))
                .addTemporalMarker(0.0, ()->robot.runCommands(new AutoLift(robot, 5, 32))) // raise lift
                .addTemporalMarker(1.0, ()->robot.runCommand(new KnockerCommand(robot, 0.05, 0.6)))
                .addTemporalMarker(0.8, ()->robot.runCommand(cbLeft))
                //.addTemporalMarker(1.5, ()->robot.runCommand(knock))
                //.addTemporalMarker(0.5,()->robot.runCommand(cbLeft))
                //.strafeLeft(2)
                .build();


        waitForStart();

        if (isStopRequested()) return;
        elementPos = od.ssIndex(20);
        Log.v("delay","done scanning");

        // hold preload
        robot.scoringSystem.claw.closeClaw();
        Log.v("delay","claw is closed" );
        // Move forward two tile
        Log.v("delay","started splining");

        robot.runCommand(drivetrain.followTrajectory(traj1));
        Log.v("delay","finished splining");
        robot.runCommand(new AutoLift(robot, 5, 27));
        Log.v("delay","going down");
        robot.runCommand(new KnockerCommand(robot, 0.05, 0.5));

        robot.runCommands(clawOpen);
        robot.runCommand(new AutoLift(robot, 5, 32));

        // Back, turn, drop chainBar, and forward to line
        robot.runCommand(drivetrain.followTrajectorySequence(
                drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                        .addTemporalMarker(0.5, ()->robot.runCommands(cbDown))
                        //.addTemporalMarker(0.8, ()->robot.runCommands(liftUp1))
                        .lineTo(new Vector2d(HI_POLE_X-6, 0))
                        .turn(Math.toRadians(87))
                        //.forward(12)
                        .build()
        ));

        robot.runCommand(drivetrain.followTrajectorySequence(
                drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                        .addTemporalMarker(0, ()->robot.runCommands(new AutoLift(robot, 5, 6)))
                        .forward(15)
                        .addTemporalMarker(0.1, ()->robot.runCommand(new KnockerCommand(robot, 0.05, 0.5)))
                        //.addTemporalMarker(0.9, ()->robot.runCommand(knock))
                        .build()
        ));

        // Follow line
        DriveTillIntake flwLine = new DriveTillIntake(robot, drivetrain,
                new Pose2d(0.1, 0, 0), 9.5, telemetry);

        robot.runCommands(flwLine);

        robot.runCommands(clawClose);

        // back to release claw


        robot.runCommand(drivetrain.followTrajectorySequence(
                drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                        //.splineTo(new Vector2d(HI_POLE_X-6, 20), Math.toRadians(-90)) // move forward
                        .back(45.5)
                        //.strafeLeft(2)
                        .addTemporalMarker(0.0, ()->robot.runCommands(new AutoLift(robot, 5, 30)))
                        .addTemporalMarker(0.5, ()->robot.runCommands(cbLeft))
                        .build()
        ));
        robot.runCommand(new AutoLift(robot, 5, 27));
        // Release cone
        robot.runCommands(clawOpen);
        robot.runCommand(new AutoLift(robot, 5, 32));

        // park
        if  (elementPos == 3) {
            robot.runCommand(drivetrain.followTrajectory(
                    drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                            //.splineTo(new Vector2d(HI_POLE_X-6, 17), Math.toRadians(-90))
                            .back(4)
                            .addTemporalMarker(0.5, ()->robot.runCommands(cbDown))
                            .addTemporalMarker(0.5, ()->robot.runCommands(new AutoLift(robot, 0, 0)))
                            .build()
            ));
        }
        if (elementPos == 2 ) {
            robot.runCommand(drivetrain.followTrajectory(
                    drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                            //.splineTo(new Vector2d(HI_POLE_X-6, 0), Math.toRadians(-90))
                            .forward(18)
                            .addTemporalMarker(1.0, ()->robot.runCommands(cbDown))
                            .addTemporalMarker(1.0, ()->robot.runCommands(new AutoLift(robot, 0, 0)))
                            .build()
            ));
            //robot.runCommands(cbDown);
            //robot.runCommands(new AutoLift(robot, 0, 0));
        }
        if (elementPos == 1 || elementPos == 4) {
            robot.runCommand(drivetrain.followTrajectory(
                    drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())

                            .forward(43)
                            .addTemporalMarker(1.0, ()->robot.runCommands(cbDown))
                            .addTemporalMarker(1.0, ()->robot.runCommands(new AutoLift(robot, 0, 0)))
                            .build()
            ));

        }
        robot.runCommands(clawOpen); // ready to pick
    }
}


