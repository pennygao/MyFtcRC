package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.subsystems.RobotColorSensor;

public class CrabRobot extends Robot {
    public final SimpleMecanumDrive mecanumDrive;
    public final Outtake outtake;
    public final RobotColorSensor robotcolorsensor;

    public CrabRobot(LinearOpMode opMode, boolean autoMode) {
        super(opMode);
        mecanumDrive = new SimpleMecanumDrive(this);
        registerSubsystem(mecanumDrive);
        outtake = new Outtake(this, autoMode, opMode.telemetry);
        robotcolorsensor = new RobotColorSensor(this, opMode.telemetry);
        registerSubsystem(robotcolorsensor);
        registerSubsystem(outtake);


    }
}
