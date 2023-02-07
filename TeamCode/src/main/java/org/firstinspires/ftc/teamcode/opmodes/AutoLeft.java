package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.AutoAlign;
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
    public static double HI_POLE_X = 54.5;

    @Override
    public void runOpMode() throws InterruptedException {
        CrabRobot robot = new CrabRobot(this,true);
        //Drivetrain3DW drivetrain = new Drivetrain3DW(robot);
        Drivetrain drivetrain = new Drivetrain(robot);
        robot.registerSubsystem((Subsystem) drivetrain);
        objectDetector od = new objectDetector(robot, telemetry);
        robot.registerSubsystem((Subsystem)od);

        // general variable
        int elementPos = 4;
        int cycles = 2;

        // Commands
        //Servo init code here
        robot.scoringSystem.SSKnockerSetPosition(0.6);
        robot.scoringSystem.chainBar.lower();
        //robot.scoringSystem.setClawPos(0.5);
        od.init();

        KnockerCommand knock = new KnockerCommand(robot, 0.05, 0.5);

        AutoCB cbRight = new AutoCB(robot, 1, 2); // auto left is -1
        AutoCB cbDown = new AutoCB(robot, 0, 2); // down is 0
        AutoClaw clawClose = new AutoClaw(robot, 0, 0.1);
        AutoClaw clawOpen = new AutoClaw(robot, 1, 0.05);
        AutoAlign autoRtCmd = new AutoAlign(robot, drivetrain,
                0.15, false, telemetry);

        Trajectory traj1 = drivetrain.trajectoryBuilder(new Pose2d())
                //.splineTo(new Vector2d(HI_POLE_X, 0), 0) // move forward
                .lineTo(new Vector2d(HI_POLE_X, 1)) // move forward
                .addTemporalMarker(0.0, ()->robot.runCommands(clawClose))
                .addTemporalMarker(0.1, ()->robot.runCommands(new AutoLift(robot, 5, 31))) // raise lift
                //.addTemporalMarker(1.0, ()->robot.runCommand(new KnockerCommand(robot, 0.05, 0.6)))
                .addTemporalMarker(1.0, ()->robot.runCommand(new KnockerCommand(robot, 0.05, 1.0)))
                .addTemporalMarker(0.8, ()->robot.runCommand(cbRight))
                .addTemporalMarker(1.5, ()->robot.runCommand(knock))
                //.addTemporalMarker(0.5,()->robot.runCommand(cbLeft))
                //.strafeLeft(2)
                .build();
        DriveTillIntake flwLine1 = new DriveTillIntake(robot, drivetrain,
                new Pose2d(0.1, 0, 0), 8.5, telemetry);
        DriveTillIntake flwLine2 = new DriveTillIntake(robot, drivetrain,
                new Pose2d(0.1, 0, 0), 5.5, telemetry);
        NanoClock clock = NanoClock.system();
        double startTime, currentTime;

        waitForStart();
        startTime = clock.seconds();
        if (isStopRequested()) return;
        Log.v("AUTODEBUG", "0: start");
        elementPos = od.ssIndex(20);
        robot.removeSubsystem((Subsystem) od);
        Log.v("AUTODEBUG", "1: OD done");
        // hold preload
        robot.scoringSystem.claw.closeClaw();
        Log.v("AUTODEBUG", "2: Hold preload done");


        // Move forward two tile
        //robot.runCommands(new AutoLift(robot, 5, 29), clawClose);
        robot.runCommand(drivetrain.followTrajectory(traj1));
        Log.v("AUTODEBUG", "3: trag1 done");

        robot.runCommands(clawOpen);
        Log.v("AUTODEBUG", "5: slide raise and claw release done");

        // Back, turn, drop chainBar, and forward to line
        robot.runCommand(drivetrain.followTrajectorySequence(
                drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                        .addTemporalMarker(0.5, ()->robot.runCommands(cbDown))
                        //.addTemporalMarker(0.8, ()->robot.runCommands(liftUp1))
                        .lineTo(new Vector2d(HI_POLE_X-4, 0))
                        .turn(Math.toRadians(88.5))
                        //.forward(12)
                        .build()
        ));
        Log.v("AUTODEBUG", "6: turn 90 done");

        robot.runCommand(drivetrain.followTrajectorySequence(
                drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                        .addTemporalMarker(0, ()->robot.runCommands(new AutoLift(robot, 5, 6)))
                        .forward(15)
                        .addTemporalMarker(0.3, ()->robot.runCommand(new KnockerCommand(robot, 0.05, 0.5)))

                        .build()
        ));
        Log.v("AUTODEBUG", "7: forward done");

        // Follow line
        robot.runCommands(flwLine1);
        Log.v("AUTODEBUG", "8: follow the line done");

        robot.runCommands(clawClose);
        Log.v("AUTODEBUG", "9: repick done");

        // back to release claw
        robot.runCommand(drivetrain.followTrajectorySequence(
                drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                        //.splineTo(new Vector2d(HI_POLE_X-6, 20), Math.toRadians(-90)) // move forward
                        .back(45)
                        //.strafeLeft(2)
                        .addTemporalMarker(0.0, ()->robot.runCommands(new AutoLift(robot, 5, 31.5)))
                        .addTemporalMarker(0.5, ()->robot.runCommands(cbRight))
                        .build()
        ));
        Log.v("AUTODEBUG", "10: drive to pole done");

        // Release cone
        robot.runCommands(clawOpen);
        Log.v("AUTODEBUG", "12: release cone done");

        if (elementPos == 1 || elementPos == 2) {
            cycles = 1;
        }
        for (int i=1; i<=cycles; i++) {
            AutoLift liftUpCmd = new AutoLift(robot, 5, 30.5);
            AutoLift liftDnCmd = new AutoLift(robot, 5, 6-i);
            robot.runCommand(drivetrain.followTrajectorySequence(
                    drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                            .forward(38)//-i*0.5)
                            //.forward(44.5-i*0.5)
                            .addTemporalMarker(1.0, ()->robot.runCommands(cbDown))
                            .addTemporalMarker(1.0, ()->robot.runCommands(liftDnCmd))
                            .build()
            ));

            robot.runCommand(flwLine2);

            robot.runCommands(clawClose);
            Log.v("AUTODEBUG", (12 + 2*i - 1) + ": repick done");
            robot.runCommand(drivetrain.followTrajectorySequence(
                    drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                            //.splineTo(new Vector2d(HI_POLE_X-6, 20), Math.toRadians(-90)) // move forward
                            .back(46)//+i*0.5)
                            //.strafeLeft(2)
                            .addTemporalMarker(0.0, ()->robot.runCommands(liftUpCmd))
                            .addTemporalMarker(0.5, ()->robot.runCommands(cbRight))
                            .build()
            ));
            robot.runCommands(clawOpen);
            Log.v("AUTODEBUG", (12 + 2*i) + ": release cone done");
            currentTime = clock.seconds();
            if (30 - (currentTime-startTime) <= 6.5){
                break;
            }
        }

        // park
        if  (elementPos == 3 || elementPos == 4) {
            robot.runCommand(drivetrain.followTrajectory(
                    drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                            //.splineTo(new Vector2d(HI_POLE_X-6, 17), Math.toRadians(-90))
                            .back(3)
                            .addTemporalMarker(0.5, ()->robot.runCommands(cbDown))
                            .addTemporalMarker(0.5, ()->robot.runCommands(new AutoLift(robot, 5, 0)))
                            .build()
            ));
        }
        if (elementPos == 2 ) {
            robot.runCommand(drivetrain.followTrajectory(
                    drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                            //.splineTo(new Vector2d(HI_POLE_X-6, 0), Math.toRadians(-90))
                            .forward(20)
                            .addTemporalMarker(1.0, ()->robot.runCommands(cbDown))
                            .addTemporalMarker(1.0, ()->robot.runCommands(new AutoLift(robot, 5, 0)))
                            .build()
            ));

        }
        if (elementPos == 1) {
            robot.runCommand(drivetrain.followTrajectory(
                    drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                            //.splineTo(new Vector2d(HI_POLE_X-6, -23), Math.toRadians(-90))
                            .forward(44)
                            .addTemporalMarker(1.0, ()->robot.runCommands(cbDown))
                            .addTemporalMarker(1.0, ()->robot.runCommands(new AutoLift(robot, 5, 0)))
                            .build()
            ));

        }
        Log.v("AUTODEBUG", "14: park done");
        robot.runCommands(clawOpen); // ready to pick

    }
}
