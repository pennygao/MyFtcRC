package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.autoLift;
import org.firstinspires.ftc.teamcode.robot.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain3DW;
import org.firstinspires.ftc.teamcode.subsystems.objectDetector;

@Autonomous
public class AutoLeftTrial extends LinearOpMode {
    public static double HI_POLE_X = 52.5;
    public static double MID_POLE_X = 26;
    public static double HI_POLE_SIDE = 14;
    public static double HI_POLE_FWD = 6;
    public static double HI_POLE_HEADING = Math.toRadians(40); // degree
    public static double POLE_HT = 44.0 ;
    public static double CONE_HT = 12;

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
        int elementPos = 4;

        //Servo init code here
        robot.outtake.setRollerPower(0.5);
        od.init();
        waitForStart();
        if (isStopRequested()) return;

        elementPos = od.ssIndex(50);



        // hold preload
        robot.runCommand(robot.outtake.rollerIntake(intakePower, 0.3));

        // Move forward then right to high pole
        autoLift liftUp = new autoLift(robot, 3, POLE_HT);
        robot.runCommand(drivetrain.followTrajectorySequence(
                drivetrain.trajectorySequenceBuilder(new Pose2d())
                        .forward(HI_POLE_X) // move forward
                        .addTemporalMarker(1.5, ()->robot.runCommands(liftUp)) // raise lift
                        .strafeRight(HI_POLE_SIDE) // strafe to pole
                        .forward(HI_POLE_FWD) // move forward
                        .build()
        ));

        // Release cone
        robot.runCommand(robot.outtake.rollerIntake(outtakePower, 0.1));
        // TODO: adjust power
        //go to cone
        robot.runCommand(drivetrain.followTrajectorySequence(
                drivetrain.trajectorySequenceBuilder(new Pose2d())
                        .back(HI_POLE_FWD) // back a little
                        .strafeLeft(HI_POLE_SIDE)
                        .addTemporalMarker(0.3,
                                ()->robot.runCommands(new autoLift(robot, 1, CONE_HT))) // retract lift
                        .turn(Math.toRadians(92))
                        .forward(28)
                        .build()
        ));
        //lower and pick up
        autoLift liftdown2 = new autoLift(robot, 0, 0);//TODO:CHANGE THE HEIGHT BASED ON THE NUMBER OF CONES
        robot.runCommands(new autoLift(robot, 1, CONE_HT-6));
        robot.runCommand(robot.outtake.rollerIntake(intakePower, 0.8));
        robot.runCommands(new autoLift(robot, 1, CONE_HT+5));


        //go back to thingy
        robot.runCommand(drivetrain.followTrajectorySequence(
                drivetrain.trajectorySequenceBuilder(new Pose2d())
                        .back(50)
                        //.turn(Math.toRadians(0 - drivetrain.getPoseEstimate().getHeading()))
                        .addTemporalMarker(1.5,
                                ()->robot.runCommands(new autoLift(robot, 3, POLE_HT+5)))
                        .strafeRight(16)
                        .forward(8)
                        .build()
        ));
        robot.runCommand(drivetrain.followTrajectorySequence(
                drivetrain.trajectorySequenceBuilder(new Pose2d())
                        .addTemporalMarker(0,()->robot.outtake.rollerIntake(outtakePower, 0.2))
                        .back(8)
                        .strafeLeft(40)
                        .addTemporalMarker(1.0,
                                ()->robot.runCommands(new autoLift(robot, 0, 0)))
                        .build()
        ));

        // park
        if  (elementPos == 1) {
            robot.runCommand(drivetrain.followTrajectorySequence(
                    drivetrain.trajectorySequenceBuilder(new Pose2d())
                            .forward(48) // move side ways
                            .build()
            ));
        } else if (elementPos == 2) {
            robot.runCommand(drivetrain.followTrajectorySequence(
                    drivetrain.trajectorySequenceBuilder(new Pose2d())
                            .forward(24) // move side ways
                            .build()
            ));

        }

    }
}