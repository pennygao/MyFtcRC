package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.autoLift;
import org.firstinspires.ftc.teamcode.robot.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain3DW;
import org.firstinspires.ftc.teamcode.subsystems.objectDetector;

@Autonomous
public class AutoRight extends LinearOpMode {
    public static double HI_POLE_X = 53.0;
    public static double HI_POLE_SIDE = 15.0;
    public static double HI_POLE_FWD = 5.5;
    public static double HI_POLE_HEADING = Math.toRadians(40); // degree
    public static double POLE_HT = 43.69;

    @Override
    public void runOpMode() throws InterruptedException {
        CrabRobot robot = new CrabRobot(this,true);
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
        robot.outtake.setRollerPower(0.5);
        od.init();
        waitForStart();
        if (isStopRequested()) return;
        elementPos = od.ssIndex(100);


        // hold preload
        robot.runCommand(robot.outtake.rollerIntake(intakePower, 0.8));

        // Move forward two tile
        robot.runCommand(drivetrain.followTrajectorySequence(
                drivetrain.trajectorySequenceBuilder(new Pose2d())
                        .forward(HI_POLE_X) // move forward
                        .strafeLeft(HI_POLE_SIDE)
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

        //go back
        robot.runCommand(drivetrain.followTrajectorySequence(
                drivetrain.trajectorySequenceBuilder(new Pose2d())
                        .strafeRight(HI_POLE_SIDE)
                        .back(26)
                        .build()
        ));

        // park
        if  (elementPos == 1) {
            int toLeft;
            toLeft = 24;
            robot.runCommand(drivetrain.followTrajectorySequence(
                    drivetrain.trajectorySequenceBuilder(new Pose2d())
                            .strafeLeft(toLeft) // move side ways
                            .build()
            ));
        }
        if (elementPos == 2 || elementPos == 4) {

            }
        if (elementPos == 3 ) {
            robot.runCommand(drivetrain.followTrajectorySequence(
                    drivetrain.trajectorySequenceBuilder(new Pose2d())
                            .strafeRight(24) // move side ways
                            .build()
            ));
        }



    }
}
