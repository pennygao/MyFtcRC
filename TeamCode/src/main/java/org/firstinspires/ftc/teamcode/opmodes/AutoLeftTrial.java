package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.autoLift;
import org.firstinspires.ftc.teamcode.robot.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.objectDetector;

@Autonomous
public class AutoLeftTrial extends LinearOpMode {
    public static double HI_POLE_X = 52.5;//26.5
    public static double MID_POLE_X = 26;
    public static double HI_POLE_SIDE = 14;
    public static double HI_POLE_FWD = 6;
    public static double HI_POLE_HEADING = Math.toRadians(40); // degree
    public static double POLE_HT = 43.69 ;
    public static double CONE_HT = 12;

    @Override
    public void runOpMode() throws InterruptedException {
        CrabRobot robot = new CrabRobot(this, true);
        Drivetrain drivetrain = new Drivetrain(robot);
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
        telemetry.addLine("eddie is bad hahahahahahahhahaaa");

        //Servo init code here
        robot.outtake.setRollerPower(0.5);
        od.init();
        waitForStart();
        if (isStopRequested()) return;

        elementPos = od.ssIndex(100);



        // hold preload
        robot.runCommand(robot.outtake.rollerIntake(intakePower, 0.8));

        // Move forward then right to high pole
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
        // TODO: adjust power

        // Back a little
        robot.runCommand(drivetrain.followTrajectorySequence(
                drivetrain.trajectorySequenceBuilder(new Pose2d())
                        .back(HI_POLE_FWD) // move forward
                        .build()
        ));

        // retract lift
        autoLift liftDown = new autoLift(robot, 0, 0);
        robot.runCommands(liftDown);

        autoLift liftUp2 = new autoLift(robot, 1, CONE_HT);
        //go to cone
        robot.runCommand(drivetrain.followTrajectorySequence(
                drivetrain.trajectorySequenceBuilder(new Pose2d())
                        .strafeLeft(HI_POLE_SIDE)
                        .turn(Math.toRadians(90 - drivetrain.getPoseEstimate().getHeading()))
                        .addTemporalMarker(1.5, ()->robot.runCommands(liftUp2))
                        .forward(25)
                        .build()
        ));
        //lower and pick up
        autoLift liftdown2 = new autoLift(robot, 0, 0);//TODO:CHANGE THE HEIGHT BASED ON THE NUMBER OF CONES
        robot.runCommands(liftdown2);
        robot.runCommand(robot.outtake.rollerIntake(intakePower, 0.8));
        robot.runCommands(liftUp2);


        //go back to thingy
        autoLift liftUp3 = new autoLift(robot, 3, POLE_HT);
        robot.runCommand(drivetrain.followTrajectorySequence(
                drivetrain.trajectorySequenceBuilder(new Pose2d())
                        .back(25)
                        .turn(Math.toRadians(0 - drivetrain.getPoseEstimate().getHeading()))
                        .addTemporalMarker(1.5,()->robot.runCommands(liftUp3))
                        .strafeRight(HI_POLE_SIDE)
                        .build()
        ));

        //forward a little
        robot.runCommand(drivetrain.followTrajectorySequence(
                drivetrain.trajectorySequenceBuilder(new Pose2d())
                        .forward(HI_POLE_FWD) // move forward
                        .build()
        ));

        // Release cone
        robot.runCommand(robot.outtake.rollerIntake(outtakePower, 0.5));
        // TODO: adjust power

        // Back a little
        robot.runCommand(drivetrain.followTrajectorySequence(
                drivetrain.trajectorySequenceBuilder(new Pose2d())
                        .back(HI_POLE_FWD) // move forward
                        .build()
        ));
        //go back
        autoLift liftDown3 = new autoLift(robot, 0, POLE_HT);
        robot.runCommand(drivetrain.followTrajectorySequence(
                drivetrain.trajectorySequenceBuilder(new Pose2d())
                        .strafeLeft(HI_POLE_SIDE)
                        .addTemporalMarker(1.5,()->robot.runCommands(liftDown3))
                        .back(26)
                        .build()
        ));

        // park
        if  (elementPos <3) {
            int toLeft;
            if (elementPos == 1)
                toLeft = 26;
            else
                toLeft = 0;
            robot.runCommand(drivetrain.followTrajectorySequence(
                    drivetrain.trajectorySequenceBuilder(new Pose2d())
                            .strafeLeft(toLeft) // move side ways
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