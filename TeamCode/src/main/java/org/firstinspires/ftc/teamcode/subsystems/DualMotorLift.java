package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Command;
import org.firstinspires.ftc.teamcode.robot.Subsystem;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import android.util.Log;

//synchronizes lifts, provides commands to move to a position
public class DualMotorLift implements Subsystem {
    //Hardware: 2 lift motors
    private DcMotorEx slideMotorL;
    private DcMotorEx slideMotorR;
    private static final double TICKS_PER_REV = 537.7; //fix this
    private static final double PULLEY_DIAMETER = 50.4 / 25.4; //update diameter of the pulley
    private final int HEIGHT_DIFF_TOLERANCE = inchToTicks(0.3); //(int) (0.3*TICKS_PER_REV / (PULLEY_DIAMETER * Math.PI));
    private Telemetry telemetry;
    private boolean targetReached = true;
    private final double  UP_VELOCITY = 500;
    public final double[] LEVEL_HT = {0, 18.0, 29.0, 39.0, 12.0}; // in inches, please fine-tune
    //4 levels: 0 ground, 1 low, 2 middle, 3 high, 4 (minimum height for free chain bar movement)
                            //0:5.0
//TODO: fine-tune LEVEL-HT values.
    public DualMotorLift (Robot robot, Telemetry telemetry){
        this.telemetry = telemetry;
        slideMotorL = robot.getMotor("slideMotorL");
        slideMotorR = robot.getMotor("slideMotorR");

        slideMotorL.setTargetPositionTolerance(HEIGHT_DIFF_TOLERANCE);
        slideMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorL.setTargetPosition(0);
        slideMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //slideMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slideMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Log.v("DualMotorSlide-motormode update", "initiating");
    }

    public void goToLevel(int level){
        //4 levels: 0 ground, 1 low, 2 middle, 3 high, 4 (minimum height for free chain bar movement)
        Log.v("DualMotorSlide-updatetarget", "goToLevel," + level + "; current status: " + targetReached);
        int targetPosition = inchToTicks(LEVEL_HT[level]);
        goToHt(targetPosition);
    }
    //for going to non-junction heights
    public void goToHt(int ticks) {
        targetReached=false;
        slideMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotorL.setTargetPosition(ticks);
        slideMotorL.setVelocity(UP_VELOCITY);
        Log.v("DualMotorSlide-motormode update", String.format("gotoHt: mode: %s, moving to: %4.2f.", slideMotorL.getMode(), ticksToInches(ticks)));
        //fine tune velocity?
        //        In case if i should set right motor right when i set left motor (prob not useful)
        //          slideMotorR.setVelocity(UP_VELOCITY);
        //          double velocity = slideMotorL.getVelocity();
        //          slideMotorR.setVelocity(velocity);
        telemetry.addData("inches:",ticksToInches(ticks));
    }

    public void adjustLift(int direction){
        //TODO: make slide motor less laggy
        slideMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotorL.setPower(0.3*direction);
        Log.v("DualMotorSlide-motormode update", String.format("adjustLift to direction: %d, power %4.2f, mode: %s", direction, slideMotorL.getPower(), slideMotorL.getMode()));
        //for using run-using-encoder mode
        //int current = slideMotorL.getCurrentPosition();
        // int targetPosition = current + inchToTicks(1)*direction;
        //slideMotorL.setTargetPosition(targetPosition);
    }

    public void stopMotor(){
        slideMotorL.setVelocity(0.0);
        slideMotorL.setPower(0);
        Log.v("DualMotorSlide-motormode update", "power off: " + slideMotorL.getPower()+" mode: " + slideMotorL.getMode());
    }

    private void updateTargetReached(){
        boolean original = this.targetReached;

        //if it is already true, don't change it. only change when slide is set to a level
        this.targetReached = (this.targetReached ||
                (Math.abs(slideMotorL.getVelocity())<=20
                    && (Math.abs(slideMotorL.getTargetPosition()-slideMotorL.getCurrentPosition())<=HEIGHT_DIFF_TOLERANCE)));
        Log.v("DualMotorSlide-updatetarget", String.format("updateTargetReached: original: %b, velocity: %4.2f, position diff: %d, pos diff tolerance: %d, targetReached set to %b. ",
                        original,
                        slideMotorL.getVelocity(),
                        slideMotorL.getTargetPosition() - slideMotorL.getCurrentPosition(),
                        HEIGHT_DIFF_TOLERANCE,
                        this.targetReached));
    }

    public boolean isLevelReached(){
        Log.v("DualMotorSlide-updatetarget", "read " + this.targetReached);
        return this.targetReached;
    }

    private int inchToTicks(double inches) {
        return (int) (inches * TICKS_PER_REV / (PULLEY_DIAMETER * Math.PI));
    }
    private double ticksToInches(int ticks){
        return ((double) ticks) / (TICKS_PER_REV / (PULLEY_DIAMETER * Math.PI));
    }

    public double getPosition(){
        return slideMotorL.getCurrentPosition() / (TICKS_PER_REV/(PULLEY_DIAMETER * Math.PI));
    }
    public boolean chainBarCanSwing(){
        return slideMotorL.getCurrentPosition()<=inchToTicks(LEVEL_HT[4]);
    }

    @Override
    public void update(TelemetryPacket packet) {

        Log.v("DualMotorSlide-updatetarget", "DualMotorLift update is called.");

        updateTargetReached();
        //TODO: Figure out why right motor is overreacting, then uncomment this
       // if(!targetReached) {
            //double velocity = slideMotorL.getVelocity();
            //slideMotorR.setVelocity(velocity);
        //}//if target is reached and not in manual mode, set velocity of right motor to 0

        telemetry.addLine("Slide motor set to " + ticksToInches(slideMotorL.getTargetPosition()));
        telemetry.addLine("current slide velocity: " + slideMotorL.getVelocity());
        telemetry.addLine("current slide position: " + ticksToInches(slideMotorL.getCurrentPosition()));
        packet.put("target pos (inches)", ticksToInches(slideMotorL.getTargetPosition()));
        packet.put("left velocity", slideMotorL.getVelocity());
        packet.put("L pos (inches)", ticksToInches(slideMotorL.getCurrentPosition()));
//        packet.put("R pos", slideMotorR.getCurrentPosition());
        packet.put("motor power", slideMotorL.getPower());
        telemetry.addLine("motor mode" + slideMotorL.getMode());
        packet.put("motor mode", slideMotorL.getMode());
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
        telemetry.update();
    }
}
