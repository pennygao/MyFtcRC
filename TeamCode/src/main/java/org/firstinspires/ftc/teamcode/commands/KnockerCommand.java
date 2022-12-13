package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.util.NanoClock;

import org.firstinspires.ftc.teamcode.robot.Command;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;

public class KnockerCommand implements Command {
    CrabRobot robot;
    double servoPos;
    NanoClock clock;
    double initialTimeStamp;
    double timeout;

    public KnockerCommand(CrabRobot robot, double pos, double duration) {// 1: knock, 0: reset
        this.robot = robot;
        servoPos = pos;
        clock = NanoClock.system();
        timeout = duration;
    }

    @Override
    public void start() {
        robot.outtake.SSKnockerSetPosition(servoPos);
        initialTimeStamp = clock.seconds();
    }

    @Override
    public void update() {
    }

    @Override
    public void stop() {
        robot.outtake.SSKnockerSetPosition(0.0);
    }

    @Override
    public boolean isCompleted() {
        return (clock.seconds() - initialTimeStamp > timeout);
    }
}
