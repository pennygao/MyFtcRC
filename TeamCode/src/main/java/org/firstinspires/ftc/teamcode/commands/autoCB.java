package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.robot.Command;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;

public class autoCB implements Command {

    CrabRobot robot;
    private double startTime;
    int direction;
    double ht;

    public autoCB(CrabRobot robot, int direction) {
        this.robot= robot;
        this.direction = direction;
        //this.ht = ht;

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
        robot.scoringSystem.dualMotorLift.adjustLift(0, true);
    }

    @Override
    public boolean isCompleted() {
        return (this.robot.scoringSystem.chainBar.doneMoving());
    }
}
