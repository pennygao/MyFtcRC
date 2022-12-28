package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.KnockerCommand;
import org.firstinspires.ftc.teamcode.commands.autoLift;
import org.firstinspires.ftc.teamcode.robot.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain3DW;
import org.firstinspires.ftc.teamcode.subsystems.objectDetector;

@Config
@Autonomous
public class AutoLeft extends LinearOpMode {
    public static double HI_POLE_X = 52.5;//26.5
    public static double MID_POLE_X = 26;
    public static double HI_POLE_SIDE = 13;
    public static double HI_POLE_FWD = 5.5;
    public static double HI_POLE_HEADING = Math.toRadians(40); // degree
    public static double POLE_HT = 44.69;

    @Override
    public void runOpMode() throws InterruptedException {
        CrabRobot robot = new CrabRobot(this, true);
        //Drivetrain drivetrain = new Drivetrain(robot);
        Drivetrain3DW drivetrain = new Drivetrain3DW(robot);
        robot.registerSubsystem((Subsystem) drivetrain);
        objectDetector od = new objectDetector(robot, telemetry);
        robot.registerSubsystem((Subsystem)od);

        // Update following parameters
        double intakePower = 0.6;
        double outtakePower = 0.1;

        // general variable
        double driveTime;
        int elementPos = 4;

        // Commands



        //Servo init code here
        robot.scoringSystem.openClaw();
        od.init();
        waitForStart();
        if (isStopRequested()) return;

        elementPos = od.ssIndex(100);

        //autoLift liftUp = new autoLift(robot, 3, POLE_HT);
        KnockerCommand knock = new KnockerCommand(robot, 0.45, 1.5);
        KnockerCommand knockerReset = new KnockerCommand(robot, 0.0, 0.0);

        autoLift liftUp = new autoLift(robot, 3, POLE_HT);

        // hold preload
        robot.scoringSystem.closeClaw();
        //robot.runCommand(robot.scoringSystem.rollerIntake(intakePower, 0.8));

        // Move forward one tile
        robot.runCommand(drivetrain.followTrajectorySequence(
                drivetrain.trajectorySequenceBuilder(new Pose2d())
                        .forward(HI_POLE_X) // move forward
                        .addTemporalMarker(0.0, ()->robot.runCommands(liftUp)) // raise lift
                        .addTemporalMarker(0.55, ()->robot.runCommand(knock))
                        .addTemporalMarker(2, ()->robot.runCommand(knockerReset))
                        .strafeRight(HI_POLE_SIDE)
                        .build()
        ));

        // Raise lift

        // Forward a little
        robot.runCommand(drivetrain.followTrajectorySequence(
                drivetrain.trajectorySequenceBuilder(new Pose2d())
                        .forward(HI_POLE_FWD) // move forward
                        .build()
        ));

        // Release cone
        robot.scoringSystem.openClaw();
        //robot.runCommand(robot.scoringSystem.rollerIntake(outtakePower, 0.5));
        // TODO: adjust power

        // Back a little
        robot.runCommand(drivetrain.followTrajectorySequence(
                drivetrain.trajectorySequenceBuilder(new Pose2d())
                        .back(HI_POLE_FWD) // move forward
                        .build()
        ));

        // retract lift
        autoLift liftDown = new autoLift(robot, 0, 0.0);
        robot.runCommands(liftDown);

        //go back
        robot.runCommand(drivetrain.followTrajectorySequence(
                drivetrain.trajectorySequenceBuilder(new Pose2d())
                        .strafeLeft(HI_POLE_SIDE)
                        .back(24)
                        .build()
        ));


        // park
        if (elementPos == 1){
            robot.runCommand(drivetrain.followTrajectorySequence(
                    drivetrain.trajectorySequenceBuilder(new Pose2d())
                            .strafeLeft(25) // move side ways
                            .build()
            ));

        } else if (elementPos == 3||elementPos == 4) {
            robot.runCommand(drivetrain.followTrajectorySequence(
                    drivetrain.trajectorySequenceBuilder(new Pose2d())
                            .strafeRight(25) // move side ways
                            .build()
            ));

        }

    }
}

