package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.util.NanoClock;

import org.firstinspires.ftc.teamcode.robot.Command;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;

public class AutoCB implements Command {

    CrabRobot robot;
    private double startTime;
    int direction;
    double duration;

    public AutoCB(CrabRobot robot, int direction, double duration) {
        this.robot= robot;
        this.direction = direction;
        this.duration = duration;
    }

    @Override
    public void start() {
        if (this.direction == 0) {
            robot.scoringSystem.chainBar.lower();
        } else {
            robot.scoringSystem.chainBar.swing_direction(this.direction);
        }
        startTime = NanoClock.system().seconds();
    }

    @Override
    public void update() {
        if (this.direction == 0) {
            robot.scoringSystem.chainBar.lower();
        } else {
            robot.scoringSystem.chainBar.swing_direction(this.direction);
        }
    }

    @Override
    public void stop() {

        //robot.scoringSystem.dualMotorLift.adjustLift(0, true);
    }

    @Override
    public boolean isCompleted() {
        //return (this.robot.scoringSystem.chainBar.doneMoving());
        return (NanoClock.system().seconds() - startTime > this.duration);
    }
}
