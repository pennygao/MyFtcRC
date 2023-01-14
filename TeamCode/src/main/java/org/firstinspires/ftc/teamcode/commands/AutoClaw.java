package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.util.NanoClock;

import org.firstinspires.ftc.teamcode.robot.Command;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;

public class AutoClaw implements Command {

    CrabRobot robot;
    private double startTime;
    int direction;
    double duration;
    NanoClock clock;

    public AutoClaw(CrabRobot robot, int direction, double duration) {
        this.robot= robot;
        this.direction = direction;
        this.duration = duration;
        clock = NanoClock.system();
    }

    @Override
    public void start() {
        startTime = clock.seconds();
        if (direction == 0)
            this.robot.scoringSystem.claw.closeClaw();
        else if (direction == 1)
            this.robot.scoringSystem.claw.openClaw();
    }

    @Override
    public void update() {

    }

    @Override
    public void stop() {
        if (direction == 0)
            this.robot.scoringSystem.claw.closeClaw();
        else if (direction == 1)
            this.robot.scoringSystem.claw.openClaw();
    }

    @Override
    public boolean isCompleted() {
        double currTime = clock.seconds();;
        return ((currTime - startTime) >= duration);
    }
}
