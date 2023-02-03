package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.robot.Command;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
import org.firstinspires.ftc.teamcode.subsystems.DualMotorLift;
import org.firstinspires.ftc.teamcode.subsystems.ScoringSystem;

public class AutoLift implements Command {

    CrabRobot robot;
    private double startTime;
    int level;
    double ht;

    public AutoLift(CrabRobot robot, int level, double ht) {
        this.robot= robot;
        this.level = level;
        this.ht = ht;
    }

    @Override
    public void start() {
        if (this.level == 0) {
            robot.scoringSystem.goAllDown();
        } else if (this.level == 5) {
            robot.scoringSystem.dualMotorLift.goToHt(robot.scoringSystem.dualMotorLift.inchToTicks(this.ht));
        } else {
            robot.scoringSystem.dualMotorLift.goToLevel(level);
        }
//        startTime = NanoClock.system().seconds();

    }

    @Override
    public void update() {
    }

    @Override
    public void stop() {

        //robot.scoringSystem.dualMotorLift.adjustLift(0, true);
        robot.scoringSystem.dualMotorLift.stopMotor();
    }

    @Override
    public boolean isCompleted() {
        return (this.robot.scoringSystem.dualMotorLift.isLevelReached());
    }
}
