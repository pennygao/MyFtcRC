package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.NanoClock;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Command;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
import org.firstinspires.ftc.teamcode.subsystems.SimpleMecanumDrive;

public class DumpFold implements Command {
    double CLAW_OPEN_TIME = 0.1; //second

    CrabRobot robot;
    SimpleMecanumDrive mecanumDrive;
    Telemetry telemetry;
    NanoClock clock;
    double xPwr;
    double timeMarker;
    int state = 0;

    public DumpFold(CrabRobot robot, SimpleMecanumDrive drive, double align_power, Telemetry telemetry){
        this.robot = robot;
        this.telemetry = telemetry;
        mecanumDrive = drive;
        clock=NanoClock.system();
        xPwr= align_power;
    }
    @Override
    public void start() {
        mecanumDrive.setDrivePower(new Pose2d(xPwr, 0, 0));
        timeMarker=clock.seconds();
        state = 0;
    }

    @Override
    public void update() {
        Log.v("AUTOCMD DEBUG"," Beginning State: " + state);
        if (state == 0) {// open claw
            robot.scoringSystem.claw.openClaw();
            if (clock.seconds() - timeMarker > CLAW_OPEN_TIME) {
                state = 1;
                timeMarker = clock.seconds();
                mecanumDrive.setDrivePower(new Pose2d(-xPwr, 0,0));
            }
        } else if (state == 1) { // Move backward for 1s
            mecanumDrive.setDrivePower(new Pose2d(-xPwr, 0,0));
            if (clock.seconds() - timeMarker >= 1.0) {
                mecanumDrive.setDrivePower(new Pose2d(0, 0,0));
                state = 2;
                timeMarker = clock.seconds();
            }
        } else if (state == 2) { //Lower chain bar, lift go to 0
            robot.scoringSystem.chainBar.lower();
            robot.scoringSystem.dualMotorLift.goToHt(0);
            timeMarker = clock.seconds();
            state = 3;
        }

        telemetry.addData("End State: ", state);
        telemetry.addData("Left dist: ", robot.robotdistancesensor.dsR);
        telemetry.addData("Rigt dist: ", robot.robotdistancesensor.dsL);
        telemetry.update();

    }

    @Override
    public void stop() {
        mecanumDrive.setDrivePower(new Pose2d(0,0,0));
    }

    @Override
    public boolean isCompleted() {
        return (   (state == 3)
        || robot.smartGamepad1.dpad_down ); // emergency stop
    }
}
