package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.NanoClock;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Command;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
import org.firstinspires.ftc.teamcode.subsystems.SimpleMecanumDrive;
import android.util.Log;

public class AutoAlign implements Command {
    double DIST_THRESHOLD = 20.0; // cm
    double CLAW_OPEN_TIME = 0.1; //second
    double POLE_OFFSET = 10.0; // cm
    double CONE_OFFSET = 3.5; // cm
    double NO_CONE_THRESHOLD = 10.0; //cm
    CrabRobot robot;
    SimpleMecanumDrive mecanumDrive;
    Telemetry telemetry;
    NanoClock clock;
    double xPwr;
    double distL, distR;
    double sideAlignDist;
    boolean alignLeft;
    double timeMarker;
    int state = 0; // 0: drive forward, till found pole 1: drop cone,
               // 2: drive backward 5 inches, 3: fold chainbar, lower slide

    //public DriveTillIntake (CrabRobot robot, SimpleMecanumDrive drive, Pose2d power, double time){
    public AutoAlign(CrabRobot robot, SimpleMecanumDrive drive, double align_power, boolean alignLeft, Telemetry telemetry){
        this.robot = robot;
        this.telemetry = telemetry;
        this.alignLeft = alignLeft;
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
        distL = robot.robotdistancesensor.dsR;
        distR = robot.robotdistancesensor.dsL;
        Log.v("AUTOCMD DEBUG"," Beginning State: " + state);
        if (state == 0) { // Move forward till in pole range
            Log.v("AUTOCMD DEBUG", "Left dist: " + robot.robotdistancesensor.dsR);
            Log.v("AUTOCMD DEBUG", "Right dist: " + robot.robotdistancesensor.dsL);
            if (       distL < DIST_THRESHOLD
                    && distR < DIST_THRESHOLD) {
                mecanumDrive.setDrivePower(new Pose2d(0, 0, 0));
                state = 1;
                timeMarker = clock.seconds();
                if (this.alignLeft) {
                    if (distL < NO_CONE_THRESHOLD) { // There is cone on left
                        sideAlignDist = POLE_OFFSET - CONE_OFFSET;
                    } else {
                        sideAlignDist = POLE_OFFSET;
                    }
                } else {// alignRight
                    if (distR < NO_CONE_THRESHOLD) { // There is cone on right
                        sideAlignDist = POLE_OFFSET - CONE_OFFSET;
                    } else {
                        sideAlignDist = POLE_OFFSET;
                    }
                }
            } else {
                mecanumDrive.setDrivePower(new Pose2d(xPwr, 0, 0));
            }
        } else if (state == 1) { // Align to the correct side
            Log.v("AUTOCMD DEBUG", "Left dist: " + robot.robotdistancesensor.dsR);
            Log.v("AUTOCMD DEBUG", "Right dist: " + robot.robotdistancesensor.dsL);
            if (this.alignLeft) {
                mecanumDrive.setDrivePower(new Pose2d(0,xPwr,0));
                if (robot.robotdistancesensor.dsR < sideAlignDist) {
                    mecanumDrive.setDrivePower(new Pose2d(0, 0,0));
                    timeMarker = clock.seconds();
                    state = 2;
                }
            } else {
                mecanumDrive.setDrivePower(new Pose2d(0,-xPwr,0));
                if (robot.robotdistancesensor.dsL < sideAlignDist) {
                    mecanumDrive.setDrivePower(new Pose2d(0, 0,0));
                    timeMarker = clock.seconds();
                    state = 2;
                }
            }
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
        return (   (state == 2)
                || robot.smartGamepad1.dpad_down ); // emergency stop
    }
}
