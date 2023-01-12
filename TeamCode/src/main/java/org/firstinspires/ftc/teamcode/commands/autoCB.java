package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.util.NanoClock;

import org.firstinspires.ftc.teamcode.robot.Command;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;

public class autoCB implements Command {

    CrabRobot robot;
    private double startTime;
    int direction;
    double duration;

    public autoCB(CrabRobot robot, int direction, double duration) {
        this.robot= robot;
        this.direction = direction;
        this.duration = duration;

    }

    @Override
    public void start() {
        robot.scoringSystem.swingChainBar(this.direction);
    }

    @Override
    public void update() {

    }

    @Override
    public void stop() {

        //robot.scoringSystem.dualMotorLift.adjustLift(0, true);
    }

    @Override
    public boolean isCompleted() {
        //return (this.robot.scoringSystem.chainBar.doneMoving());
        return (NanoClock.system().seconds() - startTime > 2);
    }
}
