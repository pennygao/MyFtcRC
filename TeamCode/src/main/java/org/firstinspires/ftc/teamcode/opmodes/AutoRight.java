package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.KnockerCommand;
import org.firstinspires.ftc.teamcode.commands.autoLift;
import org.firstinspires.ftc.teamcode.commands.autoLiftCBClaw;
import org.firstinspires.ftc.teamcode.commands.autoCB;
import org.firstinspires.ftc.teamcode.commands.autoClaw;
import org.firstinspires.ftc.teamcode.robot.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain3DW;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.objectDetector;

@Config
@Autonomous
public class AutoRight extends LinearOpMode {
    public static double HI_POLE_X = 52;
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

        // Update following parameters
        double intakePower = 0.6;
        double outtakePower = 0.1;

        // general variable
        double driveTime;
        int elementPos = 4;

        // Commands



        //Servo init code here
        robot.scoringSystem.claw.openClaw();
        robot.scoringSystem.SSKnockerSetPosition(0.4);
        //robot.scoringSystem.setClawPos(0.5);
        od.init();
        waitForStart();
        if (isStopRequested()) return;
        elementPos = od.ssIndex(100);

        KnockerCommand knock = new KnockerCommand(robot, 0.0, 1.5);
        //KnockerCommand knockerReset = new KnockerCommand(robot, 0.0, 0.0);

        autoLift liftUp1 = new autoLift(robot, 1, 38);
        autoLift liftUp3 = new autoLift(robot, 3, 38);
        autoLiftCBClaw liftUpCBClaw = new autoLiftCBClaw(robot, 3, -1, 2);
        autoCB   cbLeft = new autoCB(robot, -1, 2); // auto left is -1
        autoClaw clawClose = new autoClaw(robot, 0, 1);
        autoClaw clawOpen = new autoClaw(robot, 1, 1);

        // hold preload
        robot.scoringSystem.claw.closeClaw();
        //robot.runCommand(robot.scoringSystem.rollerIntake(intakePower, 0.8));

        // Move forward two tile
        robot.runCommand(drivetrain.followTrajectorySequence(
                drivetrain.trajectorySequenceBuilder(new Pose2d())
                        .lineTo(new Vector2d(HI_POLE_X, 0)) // move forward
                        .addTemporalMarker(0.0, ()->robot.runCommands(clawClose))
                        .addTemporalMarker(0.0, ()->robot.runCommands(liftUp3)) // raise lift
                        .addTemporalMarker(0.8, ()->robot.runCommand(knock))
                        //.addTemporalMarker(0.5,()->robot.runCommand(cbLeft))
                        .strafeLeft(2)
                        .build()
        ));
        robot.scoringSystem.swingChainBar(-1);
        robot.update();
        /*
        robot.runCommands(cbLeft);
        robot.runCommands(clawOpen);

        robot.runCommand(drivetrain.followTrajectorySequence(
                drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                        .forward(5)
                        //.addTemporalMarker(0.0, ()->robot.runCommands(clawOpen))
                        .build()
        ));
        robot.runCommand(clawOpen);

        // Forward a little
        robot.runCommand(drivetrain.followTrajectorySequence(
                drivetrain.trajectorySequenceBuilder(new Pose2d())
                        .forward(HI_POLE_FWD) // move forward
                        .build()
        ));

        // Release cone
        robot.scoringSystem.claw.openClaw();
       // robot.runCommand(robot.scoringSystem.rollerIntake(outtakePower, 0.5));
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
*/


    }
}
