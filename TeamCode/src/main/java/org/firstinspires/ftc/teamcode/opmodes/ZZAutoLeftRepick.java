package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.autoLift;
import org.firstinspires.ftc.teamcode.robot.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain3DW;
import org.firstinspires.ftc.teamcode.subsystems.objectDetector;

@Autonomous
public class ZZAutoLeftRepick extends LinearOpMode {
    public static double HI_POLE_X = 52.5;
    public static double MID_POLE_X = 26;
    public static double HI_POLE_SIDE = 12;
    public static double HI_POLE_FWD = 6;
    public static double HI_POLE_HEADING = Math.toRadians(40); // degree
    public static double POLE_HT = 46.0 ;//46.0
    public static double CONE_HT = 10;

    @Override
    public void runOpMode() throws InterruptedException {
        CrabRobot robot = new CrabRobot(this, true);
        Drivetrain3DW drivetrain = new Drivetrain3DW(robot);
        robot.registerSubsystem((Subsystem) drivetrain);
        objectDetector od = new objectDetector(robot, telemetry);
        robot.registerSubsystem((Subsystem)od);

        // Update following parameters
        double intakePower = 0.6;
        double outtakePower = 0.1;

        // general variable
        double driveTime;
        int elementPos = 3;

        //Servo init code here
        robot.scoringSystem.claw.openClaw();
        od.init();
        waitForStart();

        if (isStopRequested()) return;

        elementPos = od.ssIndex(10);



        // hold preload
        robot.scoringSystem.claw.closeClaw();

        // Move forward then right to high pole
        autoLift liftUp = new autoLift(robot, 3, POLE_HT);

        robot.runCommand(drivetrain.followTrajectorySequence(
                drivetrain.trajectorySequenceBuilder(new Pose2d())
                        //.forward(HI_POLE_X) // move forward
                        .addTemporalMarker(0.2, ()->robot.runCommands(liftUp)) // raise lift
                        .splineTo(new Vector2d(57.5, -5.8), Math.toRadians(-48)) // strafe to pole
                        //.forward(HI_POLE_FWD) // move forward
                        .build()
        ));
        // Release cone
        robot.scoringSystem.claw.openClaw();
        //robot.runCommand(robot.scoringSystem.rollerIntake(outtakePower, 0.1));
/*
        robot.runCommand(drivetrain.followTrajectorySequence(
                drivetrain.trajectorySequenceBuilder(new Pose2d())
                        .addTemporalMarker(1.0, ()->robot.runCommands(liftUp)) // raise lift
                        .forward(58) // move forward
                        .back(8)
                        .turn(Math.toRadians(-43))
                        .forward(10.5)
                        .build()
        ));
*/


        // re-pick
        robot.runCommand(drivetrain.followTrajectorySequence(
                drivetrain.trajectorySequenceBuilder(new Pose2d())
                        // retract lift
                        .addTemporalMarker(0.5,
                                ()->robot.runCommands(new autoLift(robot, 1, CONE_HT)))
                        .back(10) // move backward
                        .turn( Math.toRadians(135))
                        .strafeRight(1)
                        .forward(26.5)
                        .build()
        ));

        robot.runCommands(new autoLift(robot, 1, CONE_HT-4));
        robot.scoringSystem.claw.closeClaw();
        robot.runCommands(new autoLift(robot, 1, CONE_HT+5));


        robot.runCommand(drivetrain.followTrajectorySequence(
                drivetrain.trajectorySequenceBuilder(new Pose2d())
                        .addTemporalMarker(0.3,
                                ()->robot.runCommands(new autoLift(robot, 3, POLE_HT+3)))
                        .back(27)
                        .turn(Math.toRadians(-135))
                        .forward(9.0)
                        .build()
        ));

        // Release cone
        robot.scoringSystem.claw.openClaw();

        // Back, turn and retract lift
        robot.runCommand(drivetrain.followTrajectorySequence(
                drivetrain.trajectorySequenceBuilder(new Pose2d())
                        //.addTemporalMarker(0.3,
                        //        ()->robot.runCommands(new autoLift(robot, 0, 0)))
                        .back(10)
                        .turn(Math.toRadians(48))
                        .build()
        ));

        // park
        if  (elementPos == 1) {
            robot.runCommand(drivetrain.followTrajectorySequence(
                    drivetrain.trajectorySequenceBuilder(new Pose2d())
                            .strafeLeft(24) // move side ways
                            .build()
            ));
        } else if (elementPos == 3) {
            robot.runCommand(drivetrain.followTrajectorySequence(
                    drivetrain.trajectorySequenceBuilder(new Pose2d())
                            .strafeRight(24) // move side ways
                            .build()
            ));

        }



    }
}