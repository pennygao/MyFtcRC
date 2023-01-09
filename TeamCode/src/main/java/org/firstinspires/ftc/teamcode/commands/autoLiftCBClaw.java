package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.robot.Command;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
import org.intellij.lang.annotations.JdkConstants;

import com.acmerobotics.roadrunner.util.NanoClock;

public class autoLift implements Command {

    CrabRobot robot;
    private double startTime;
    int level;
    double duration;
    int CBDir;

    public autoLift(CrabRobot robot, int level, int CBDir, double duration) {
        this.robot= robot;
        this.level = level;
        this.duration = duration;
        this.CBDir = CBDir;
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
        if (this.robot.scoringSystem.dualMotorLift.isLevelReached()) {
            startTime = NanoClock.system().seconds();
            robot.scoringSystem.swingChainBar(this.CBDir);
        }
        if (this.robot.scoringSystem.chainBar.doneMoving()) {
            startTime = NanoClock.system().seconds();
            robot.scoringSystem.claw.openClaw();
        }
    }

    @Override
    public void stop() {
        robot.scoringSystem.dualMotorLift.adjustLift(0, true);
    }

    @Override
    public boolean isCompleted() {
        //return (this.robot.scoringSystem.dualMotorLift.isLevelReached());
        return ((NanoClock.system().seconds() - startTime) > duration);
    }
}
