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

    public RobotColorSensor(Robot robot, Telemetry telemetry) {
        ColorSensorL = robot.getcolorSensor("ColorL");
        ColorSensorR = robot.getcolorSensor("ColorR");
        this.telemetry = telemetry;



    }
    @Override
    public void update(TelemetryPacket packet) {


        telemetry.addData("RedL:", ColorSensorL.red());
        telemetry.addData("GreenL:", ColorSensorL.green());
        telemetry.addData("BlueL:", ColorSensorL.blue());
        telemetry.addData("RedR:", ColorSensorR.red());
        telemetry.addData("GreenR:", ColorSensorR.green());
        telemetry.addData("BlueR:", ColorSensorR.blue());
        telemetry.update();


        }
        // debug only,  remove it on release
        // packet.put("Current Position", slideMotor.getCurrentPosition());
        //  packet.put("target position", slideMotor.getTargetPosition());
    }




