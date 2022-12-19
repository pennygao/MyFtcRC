package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.DriveTillIntake;
import org.firstinspires.ftc.teamcode.commands.KnockerCommand;
import org.firstinspires.ftc.teamcode.commands.autoLift;
import org.firstinspires.ftc.teamcode.robot.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain3DW;
import org.firstinspires.ftc.teamcode.subsystems.objectDetector;

@Autonomous
public class ZZAutoRightRepick extends LinearOpMode {
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
        robot.outtake.setRollerPower(0.5);
        robot.outtake.SSKnockerSetPosition(0.0);
        od.init();
        waitForStart();

        if (isStopRequested()) return;

        elementPos = od.ssIndex(10);


        // hold preload
        //robot.runCommand(robot.outtake.rollerIntake(intakePower, 0.5));
        robot.runCommand(robot.outtake.rollerIntake(intakePower, 0.2));

        // Move forward then right to high pole
        autoLift liftUp = new autoLift(robot, 3, POLE_HT);
        KnockerCommand knock = new KnockerCommand(robot, 0.45, 1.5);
        KnockerCommand knockerReset = new KnockerCommand(robot, 0.0, 0.0);

        robot.runCommand(drivetrain.followTrajectorySequence(
                drivetrain.trajectorySequenceBuilder(new Pose2d())
                        .addTemporalMarker(0.0, ()->robot.runCommands(liftUp)) // raise lift
                        .addTemporalMarker(0.7, ()->robot.runCommand(knock))
                        .addTemporalMarker(1.5, ()->robot.runCommand(knockerReset))
                        .splineTo(new Vector2d(60, 8), Math.toRadians(48)) // strafe to pole
                        .build()
        ));
        // Release cone
        robot.runCommand(robot.outtake.rollerIntake(outtakePower, 0.4));


        // re-pick
        robot.runCommand(drivetrain.followTrajectorySequence(
                drivetrain.trajectorySequenceBuilder(new Pose2d())
                        // retract lift
                        .addTemporalMarker(0.5,
                                ()->robot.runCommands(new autoLift(robot, 1, CONE_HT)))
                        .back(9.5) // move backward
                        .turn( Math.toRadians(-134))
                        //.strafeRight(1)
                        .forward(11)
                        .build()
        ));

        // Follow line
        DriveTillIntake flwLine = new DriveTillIntake(robot, drivetrain,
                new Pose2d(0.1, 0, 0), 3.3, telemetry);
        robot.runCommand(flwLine);

        robot.runCommands(new autoLift(robot, 1, CONE_HT-5));
        robot.runCommand(robot.outtake.rollerIntake(intakePower, 1.0));
        robot.runCommands(new autoLift(robot, 1, CONE_HT+5));


        robot.runCommand(drivetrain.followTrajectorySequence(
                drivetrain.trajectorySequenceBuilder(new Pose2d())
                        .addTemporalMarker(0.3, ()->robot.runCommands(new autoLift(robot, 3, POLE_HT)))
                        .back(26)
                        .turn(Math.toRadians(121))
                        .forward(11.0)

                        .build()
        ));



        // Release cone
        robot.runCommand(robot.outtake.rollerIntake(outtakePower, 0.1));

        // Back, turn and retract lift
        robot.runCommand(drivetrain.followTrajectorySequence(
                drivetrain.trajectorySequenceBuilder(new Pose2d())
                        .addTemporalMarker(0.3,
                                ()->robot.runCommands(new autoLift(robot, 0, 0)))
                        .back(9)
                        .turn(Math.toRadians(-35))
                        .build()
        ));

        // park
        if  (elementPos == 1) {
            robot.runCommand(drivetrain.followTrajectorySequence(
                    drivetrain.trajectorySequenceBuilder(new Pose2d())
                            .strafeLeft(24) // move side ways
                            .build()
            ));
        } else if (elementPos == 3 || elementPos == 4) {
            robot.runCommand(drivetrain.followTrajectorySequence(
                    drivetrain.trajectorySequenceBuilder(new Pose2d())
                            .strafeRight(24) // move side ways
                            .build()
            ));

        }


    }
}