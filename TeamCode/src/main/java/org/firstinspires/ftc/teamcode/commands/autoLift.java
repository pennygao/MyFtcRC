package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.util.NanoClock;

import org.firstinspires.ftc.teamcode.robot.Command;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;

public class autoLift implements Command {

    CrabRobot robot;
    private double startTime;
    int level;

    public autoLift(CrabRobot robot, int level) {
        this.robot= robot;
        this.level = level;
    }

    @Override
    public void start() {
        robot.outtake.goToLevel(this.level);
        startTime = NanoClock.system().seconds();
        robot.update();
    }

    @Override
    public void update() {

    }

    @Override
    public void stop() {

    }

    @Override
    public boolean isCompleted() {

        return (!this.robot.outtake.slideMotorBusy());


    }
}
