package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.robot.Command;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;

public class autoLiftDown implements Command {

    CrabRobot robot;
    private double startTime;

    public autoLiftDown(CrabRobot robot) {
        this.robot= robot;
    }

    @Override
    public void start() {
         robot.scoringSystem.goAllDown();
         robot.update();
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
