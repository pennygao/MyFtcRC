package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.autoIntake;
import org.firstinspires.ftc.teamcode.commands.autoLift;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;

@Autonomous
public class AAAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CrabRobot robot = new CrabRobot(this);

        // Update following parameters
        double intakePower = 0.6;
        double outtakePower = 0;

        // general variable
        double driveTime;

        //Servo init code here
        robot.outtake.setRollerPower(0.5);


        waitForStart();

        // hold preload
        driveTime = 2.5;
        autoIntake spinIn = new autoIntake(robot, intakePower, driveTime);
        robot.runCommands(spinIn);

        // Raise lift
        autoLift liftUp = new autoLift(robot, 2);
        robot.runCommands(liftUp);

        // drop cone
        autoIntake spinOut = new autoIntake(robot, outtakePower, driveTime);
        robot.runCommand(spinOut);

        // retract lift
        autoLift liftDown = new autoLift(robot, 0);
        robot.runCommands(liftDown);
        /*
       // go forwards slightly
        Pose2d drivePower = new Pose2d(-0.2,0,0);
        driveTime = 0.52;
        DriveForTime driveCommand = new DriveForTime(robot.mecanumDrive, drivePower, driveTime);
        robot.runCommand(driveCommand);

        //strafe
        Pose2d sideways = new Pose2d(0,0.4,0);
        driveTime = 2.5;
        DriveForTime goSideways = new DriveForTime(robot.mecanumDrive, sideways, driveTime);
        robot.runCommand(goSideways);

        //strafe0
        Pose2d sideways0 = new Pose2d(0,0.1,0);
        driveTime = 0.5;
        DriveForTime goSideways0 = new DriveForTime(robot.mecanumDrive, sideways0, driveTime);
        robot.runCommand(goSideways0);



        //sickstrafes2
        Pose2d sideways2 = new Pose2d(0,-0.5,0);
        driveTime = 0.75;
        DriveForTime goSideways2 = new DriveForTime(robot.mecanumDrive, sideways2, driveTime);
        robot.runCommand(goSideways2);

        //waitaminute
        Pose2d nopower = new Pose2d(0,0,0);
        driveTime = 2;
        DriveForTime drivenoCommand = new DriveForTime(robot.mecanumDrive, nopower, driveTime);
        robot.runCommand(drivenoCommand);

        //sickstrafes3
        Pose2d sideways3 = new Pose2d(0,-0.5,0);
        driveTime = 1.5;
        DriveForTime goSideways3 = new DriveForTime(robot.mecanumDrive, sideways3, driveTime);
        robot.runCommand(goSideways3);



         */



        // turn 90 towards warehouse
        /*double turnAngle = Math.toRadians(87);
        Turn  turnCommand = new Turn(robot.mecanumDrive, turnAngle);
        robot.runCommand(turnCommand);

        // back to carousal
        Pose2d carousel = new Pose2d(-0.25,0,0);
        driveTime = 1.65;
        DriveForTime backToCarousel = new DriveForTime(robot.mecanumDrive, carousel, driveTime);
        robot.runCommand(backToCarousel);
        */


//obama
    }
}

