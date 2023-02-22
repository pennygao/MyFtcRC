package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.NanoClock;

//import org.firstinspires.ftc.teamcode.Configuration;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Command;
import org.firstinspires.ftc.teamcode.robot.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain3DW;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.SimpleMecanumDrive;

public class DriveTillIntake implements Command {
    CrabRobot robot;
    //SimpleMecanumDrive mecanumDrive;
    Drivetrain mecanumDrive;
    Telemetry telemetry;
    NanoClock clock;
    Pose2d drivePower;
    double startX, startY;
    double xPwr;
    double coef = 0.5;
    double hcoef = 0.23;
    double initialTimeStamp, intakeCompleteTime;
    double driveDisp;

    //public DriveTillIntake (CrabRobot robot, SimpleMecanumDrive drive, Pose2d power, double time){
    public DriveTillIntake (CrabRobot robot, Drivetrain drive, Pose2d power, double xDisp, Telemetry telemetry){
        this.robot = robot;
        this.telemetry = telemetry;
        mecanumDrive = drive;
        clock=NanoClock.system();
        drivePower= power;
        xPwr = drivePower.getX();
        driveDisp = xDisp;
    }
    @Override
    public void start() {
        mecanumDrive.setDrivePower(drivePower);
        robot.registerSubsystem((Subsystem) robot.robotcolorsensor);
        initialTimeStamp=clock.seconds();
        startX = mecanumDrive.getPoseEstimate().getX();
        startY = mecanumDrive.getPoseEstimate().getY();
    }

    @Override
    public void update() {

        // L online, R not online => turn left
        if (robot.robotcolorsensor.csLonLine() && !robot.robotcolorsensor.csRonLine()) {
            mecanumDrive.setDrivePower(new Pose2d(xPwr,xPwr*coef ,-hcoef*xPwr));
        } else if (!robot.robotcolorsensor.csLonLine() && robot.robotcolorsensor.csRonLine()) {
            mecanumDrive.setDrivePower(new Pose2d(xPwr, -xPwr*coef, hcoef*xPwr));
        } else {
            mecanumDrive.setDrivePower(drivePower);
        }

    }

    @Override
    public void stop() {
        mecanumDrive.setDrivePower(new Pose2d(0,0,0));
        robot.removeSubsystem((Subsystem) robot.robotcolorsensor);
    }

    @Override
    public boolean isCompleted() {
        //double currTime=clock.seconds();
        //return (currTime - initialTimeStamp >=driveDisp); // || currTime - intakeCompleteTime >= 1);

        Pose2d currPos = mecanumDrive.getPoseEstimate();
        double disp = Math.abs(currPos.getY() - startY);
        telemetry.addData("currX: ", currPos.getX());
        telemetry.addData("currY: ", currPos.getY());
        telemetry.addData("startX: ", startX);
        telemetry.addData("startY: ", startY);
        telemetry.addData("disp: ", disp);
        telemetry.update();
        //return (disp >= driveDisp);
        return robot.scoringSystem.distC.coneReached();


    }
}
