package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Subsystem;


public class DistClaw implements Subsystem {
    private DistanceSensor DistClaw;
    private Telemetry telemetry;
    public double dsC;

    public DistClaw(Robot robot, Telemetry telemetry) {
        DistClaw = robot.getHardwareMap().get(DistanceSensor.class, "DistClaw");
        this.telemetry = telemetry;
        dsC = 0;

    }

    public double getDist(){
        return DistClaw.getDistance(DistanceUnit.CM);
    }

    @Override
    public void update(TelemetryPacket packet) {
        dsC = getDist();
    }
}