package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Subsystem;



public class RobotColorSensor implements Subsystem {
    private ColorSensor ColorSensorL;
    private ColorSensor ColorSensorR;
    private Telemetry telemetry;
    public double[] csL;
    public double[] csR;
    private double threashold = 0.65;

    public RobotColorSensor(Robot robot, Telemetry telemetry) {
        ColorSensorL = robot.getcolorSensor("ColorL");
        ColorSensorR = robot.getcolorSensor("ColorR");
        this.telemetry = telemetry;
        csL = new double[3];
        csR = new double[3];

    }
    public boolean csLonLine() {
        return ((csL[0]+csL[2])/(csL[0]+csL[1]+csL[2]) > this.threashold);
    }
    public boolean csRonLine() {
        return ((csR[0] + csR[2]) / (csR[0] + csR[1] + csR[2]) > this.threashold);
    }
    @Override
    public void update(TelemetryPacket packet) {

        csL[0] = ColorSensorL.red();
        csL[1] = ColorSensorL.green();
        csL[2] = ColorSensorL.blue();
        csR[0] = ColorSensorR.red();
        csR[1] = ColorSensorR.green();
        csR[2] = ColorSensorR.blue();
    }
}




