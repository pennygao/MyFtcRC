package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.NanoClock;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Command;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.SimpleMecanumDrive;
import android.util.Log;

public class DriveRaiseDumpFold implements Command {
    double DIST_THRESHOLD = 20.0; // cm
    double CLAW_OPEN_TIME = 0.1; //second
    double POLE_OFFSET = 8.0; // cm
    CrabRobot robot;
    SimpleMecanumDrive mecanumDrive;
    Telemetry telemetry;
    NanoClock clock;
    double xPwr;
    boolean alignLeft;
    double timeMarker;
    int state = 0; // 0: drive forward, till found pole 1: drop cone,
               // 2: drive backward 5 inches, 3: fold chainbar, lower slide

    //public DriveTillIntake (CrabRobot robot, SimpleMecanumDrive drive, Pose2d power, double time){
    public DriveRaiseDumpFold(CrabRobot robot, SimpleMecanumDrive drive, double align_power, boolean alignLeft, Telemetry telemetry){
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
        Log.v("AUTOCMD DEBUG"," Beginning State: " + state);
        if (state == 0) { // Move forward till in pole range
            Log.v("AUTOCMD DEBUG", "Left dist: " + robot.robotdistancesensor.dsR);
            Log.v("AUTOCMD DEBUG", "Right dist: " + robot.robotdistancesensor.dsL);
            if (       robot.robotdistancesensor.dsL < DIST_THRESHOLD
                    && robot.robotdistancesensor.dsR < DIST_THRESHOLD) {
                mecanumDrive.setDrivePower(new Pose2d(0, 0, 0));
                state = 1;
                timeMarker = clock.seconds();
            } else {
                mecanumDrive.setDrivePower(new Pose2d(xPwr, 0, 0));
            }
        } else if (state == 1) { // Align to the correct side
            Log.v("AUTOCMD DEBUG", "Left dist: " + robot.robotdistancesensor.dsR);
            Log.v("AUTOCMD DEBUG", "Right dist: " + robot.robotdistancesensor.dsL);
            if (this.alignLeft) {
                mecanumDrive.setDrivePower(new Pose2d(0,xPwr,0));
                if (robot.robotdistancesensor.dsR < POLE_OFFSET) {
                    mecanumDrive.setDrivePower(new Pose2d(0, 0,0));
                    timeMarker = clock.seconds();
                    state = 2;
                }
            } else {
                mecanumDrive.setDrivePower(new Pose2d(0,-xPwr,0));
                if (robot.robotdistancesensor.dsL < POLE_OFFSET) {
                    mecanumDrive.setDrivePower(new Pose2d(0, 0,0));
                    timeMarker = clock.seconds();
                    state = 2;
                }
            }
        } else if (state == 2) { // open claw
            robot.scoringSystem.claw.openClaw();
            if (clock.seconds() - timeMarker > CLAW_OPEN_TIME) {
                state = 3;
                timeMarker = clock.seconds();
                mecanumDrive.setDrivePower(new Pose2d(-0.7, 0,0));
            }
        } else if (state == 3) { // Move backward for 1s
            mecanumDrive.setDrivePower(new Pose2d(-0.7, 0,0));
            if (clock.seconds() - timeMarker >= 1.0) {
                mecanumDrive.setDrivePower(new Pose2d(0, 0,0));
                state = 4;
                timeMarker = clock.seconds();
            }
        } else if (state == 4) { //Lower chain bar, lift go to 0
            robot.scoringSystem.chainBar.lower();
            robot.scoringSystem.dualMotorLift.goToHt(0);
            timeMarker = clock.seconds();
            state = 5;
        } if (state == 5) { // wait for end
            Log.v("AUTOCMD DEBUG"," time " + (clock.seconds() - timeMarker));
            Log.v("AUTOCMD DEBUG"," Lift reached " + this.robot.scoringSystem.dualMotorLift.isLevelReached());
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
        return (   (state == 5
                && this.robot.scoringSystem.dualMotorLift.isLevelReached()
                && (clock.seconds() - timeMarker >= 0.5))
        || robot.smartGamepad1.dpad_down ); // emergency stop
    }
}
