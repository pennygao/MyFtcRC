package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.util.NanoClock;

import org.firstinspires.ftc.teamcode.robot.Command;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;

public class autoAdjustLift implements Command {

    CrabRobot robot;
    private double startTime;
    private boolean down;

    public autoAdjustLift(CrabRobot robot, boolean down) {
        this.robot= robot;
        this.down = down;
    }

    @Override
    public void start() {
        if (down) {
            robot.scoringSystem.dualMotorLift.adjustLift(-1);
        } else {
            robot.scoringSystem.dualMotorLift.adjustLift(1);
        }
        startTime = NanoClock.system().seconds();
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
