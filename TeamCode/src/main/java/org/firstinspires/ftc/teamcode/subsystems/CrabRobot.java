package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Robot;

public class CrabRobot extends Robot {
    public final SimpleMecanumDrive mecanumDrive;
    public final Outtake outtake;
    public CrabRobot(LinearOpMode opMode, boolean autoMode) {
        super(opMode);
        mecanumDrive = new SimpleMecanumDrive(this);
        registerSubsystem(mecanumDrive);
        outtake = new Outtake(this, autoMode , opMode.telemetry);
        registerSubsystem(outtake);


}
}
