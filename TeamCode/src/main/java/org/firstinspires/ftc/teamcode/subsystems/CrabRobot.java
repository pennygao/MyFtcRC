package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Robot;

public class CrabRobot extends Robot {
    public final SimpleMecanumDrive mecanumDrive;
    public final ScoringSystem scoringSystem;
    public final RobotColorSensor robotcolorsensor;

    public CrabRobot(LinearOpMode opMode, boolean autoMode) {
        super(opMode);
        mecanumDrive = new SimpleMecanumDrive(this);
        registerSubsystem(mecanumDrive);
        scoringSystem = new ScoringSystem(this, autoMode, opMode.telemetry);
        robotcolorsensor = new RobotColorSensor(this, opMode.telemetry);
        registerSubsystem(robotcolorsensor);
        registerSubsystem(scoringSystem);

    }
}
