package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.util.NanoClock;

import org.firstinspires.ftc.teamcode.robot.Command;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;

public class autoLiftCBClaw implements Command {

    CrabRobot robot;
    private double startTime;
    int level;
    double duration;
    int CBDir;

    public autoLiftCBClaw(CrabRobot robot, int level, int CBDir, double duration) {
        this.robot= robot;
        this.level = level;
        this.duration = duration;
        this.CBDir = CBDir;
    }

    @Override
    public void start() {
        robot.scoringSystem.swingChainBar(this.CBDir);
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
        robot.scoringSystem.dualMotorLift.adjustLift(0, true);
    }

    @Override
    public boolean isCompleted() {
        return (this.robot.scoringSystem.dualMotorLift.isLevelReached() &&
                this.robot.scoringSystem.chainBar.doneMoving()
        );

    }
}
