package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.robot.Command;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;

public class autoLift implements Command {

    CrabRobot robot;
    private double startTime;
    int level;
    double ht;

    public autoLift(CrabRobot robot, int level, double ht) {
        this.robot= robot;
        this.level = level;
        this.ht = ht;

    }

    @Override
    public void start() {
        if (this.level == 0) {
            robot.scoringSystem.goAllDown();
            robot.update();
        } else {
            robot.scoringSystem.dualMotorLift.goToLevel(level);
            robot.update();
        }
//        startTime = NanoClock.system().seconds();

    }

    @Override
    public void update() {

    }

    @Override
    public void stop() {
        robot.scoringSystem.dualMotorLift.adjustLift(0);
    }

    @Override
    public boolean isCompleted() {
        return (!this.robot.scoringSystem.dualMotorLift.isLevelReached());
    }
}
