package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.util.NanoClock;

import org.firstinspires.ftc.teamcode.robot.Command;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;

public class autoLiftInch implements Command {

    CrabRobot robot;
    private double startTime;
    private boolean down;

    public autoLiftInch(CrabRobot robot, boolean down) {
        this.robot= robot;
        this.down = down;
    }

    @Override
    public void start() {
        if (down) {
            robot.outtake.goDown1Inch();
        } else {
            robot.outtake.goUp1Inch();
        }
        startTime = NanoClock.system().seconds();
        robot.update();
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
