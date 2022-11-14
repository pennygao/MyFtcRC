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
         robot.outtake.goalldown();
         robot.update();
//        startTime = NanoClock.system().seconds();

    }

    @Override
    public void update() {

    }

    @Override
    public void stop() {
        robot.outtake.slideMotor.setPower(0);
    }

    @Override
    public boolean isCompleted() {

        return (!this.robot.outtake.slideMotorBusy());


    }
}
