package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Subsystem;
import com.acmerobotics.roadrunner.util.NanoClock;

import android.util.Log;

@Config
public class ScoringSystem implements Subsystem {
    //Hardware: 1 motor, 1 encoder
    public static final double TICKS_PER_REV = 537.7*(11.0/15.0);
    public static final double PULLEY_DIAMETER = 38.0 / 25.4;
    public DualMotorLift dualMotorLift;
    public Telemetry telemetry;

    private int swingState = 0;
    private int lowerState = 0;

    private Servo SSKnocker;
    private double ssKnockerPos = 0;

    private ChainBar chainBar;
    private Claw claw;
    //TODO: tune
    public static final double CLAW_OPEN_POSITION = 0.5;
    public static final double CLAW_CLOSE_POSITION = 0.1;

    public class Claw {
        public Servo clawServo;
        NanoClock clock;
        double initialTimestamp;
        double movementTime;
        double toPos;

        public Claw(Robot robot){
            clawServo = robot.getServo("claw");
        }
        public void setClawPos(double pos) {
            toPos = pos;
            clawServo.setPosition(pos);
        }
        public double getClawTarget(){
            return clawServo.getPosition();
        }
        public boolean isCompleted() {
            return clock.seconds() - initialTimestamp >= movementTime;
        }
    }

    public class ChainBar{
        public Servo cbServo;
        private double lastMovement=0;
        //TODO: tune these values to actual chain bar
        private final double CHAIN_BAR_DOWN = 0.5;
        private final double CHAIN_BAR_UP = 0.1; //How much you move the chain bar up from down position
        private int CBDirection = 0;

        public ChainBar(Robot robot){
            cbServo = robot.getServo("chainBar");
        }

        public void swing(){
            cbServo.setPosition(CHAIN_BAR_DOWN + CHAIN_BAR_UP*CBDirection);
            lastMovement = NanoClock.system().seconds();
        }
        public void lower(){
            cbServo.setPosition(CHAIN_BAR_DOWN);
            lastMovement = NanoClock.system().seconds();
        }
        public void changeDirection (int direction){
            CBDirection = direction;
        }
        public boolean doneMoving(){
            return (NanoClock.system().seconds() - lastMovement > 1);
        }
        public void adjust(int direction){
            double toPosition = cbServo.getPosition(); //get target position
            cbServo.setPosition(toPosition+(direction*0.00625));
        }
    }

    public ScoringSystem(Robot robot, boolean autoMode, Telemetry telemetry) {
        this.telemetry = telemetry;
        claw = new Claw(robot);
        chainBar = new ChainBar(robot);
        SSKnocker = robot.getServo("SSKnocker");
        dualMotorLift = new DualMotorLift(robot, telemetry, DualMotorLift.Mode.BOTH_MOTORS_PID);
        telemetry.update();

    }

    public int inchToTicks(double inches) {
        return (int) (inches * TICKS_PER_REV / (PULLEY_DIAMETER * Math.PI));
    }

    public void adjustLift(int direction){
        dualMotorLift.adjustLift(direction);
    }

    public void goAllDown() {
        lowerState = 1;
        dualMotorLift.goToLevel(4);   //go to level that's safe for lowering chain bar
        Log.v("DualMotorSlide-updatetaret", "gotoLevel(4) get called in goAllDown()");
    }

    public boolean isLiftLevelReached(){
        return dualMotorLift.isLevelReached();
    }

    public void swingChainBar(int direction){
        swingState = 1;
        chainBar.changeDirection(direction);
    }

    public void adjustChainBar(int direction){
        chainBar.adjust(direction);
    }

    public void SSKnockerSetPosition(double pos) {
        ssKnockerPos = pos;
    }

    public void openClaw(){
        claw.setClawPos(CLAW_OPEN_POSITION);
    }
    public void closeClaw(){
        claw.setClawPos(CLAW_CLOSE_POSITION);
    }


    @Override
    public void update(TelemetryPacket packet) {
        dualMotorLift.update(packet);
        Log.v("DualMotorSlide-updatetarget", "ScoringSystem update is called.");
        //case for how to lower chain bar :wah:
        /*
         * 0: chain bar does not need to raise, skip this sequence if so.
         * 1: not yet reached height, wait.
         * 2: height reached, swing chain bar.
         */
        if(swingState==1){
            Log.v("ChainBar", dualMotorLift.chainBarCanSwing()+"");
            if(!dualMotorLift.chainBarCanSwing()){
                dualMotorLift.goToLevel(4);
                Log.v("DualMotorSlide- ChainBar", "gotoLevel(4) get called");
            }
            swingState=2;
            telemetry.addLine("CB: waiting to raise slide...");
            Log.v("ChainBar:", "waiting to raise slide...");
        }
        else if(swingState==2){
            if(dualMotorLift.chainBarCanSwing()) {
                chainBar.swing();
                swingState = 0;
                telemetry.addLine("CB: slide is able to swing");
                Log.v("ChainBar:", "slide is able to swing");
            }
        }

        if(lowerState==1){
            chainBar.lower();
            lowerState++;
            telemetry.addLine("CB: waiting for chain bar to lower...");
            Log.v("ChainBar:", "waiting for chain bar to lower...");
        }
        else if(lowerState==2){
            if(chainBar.doneMoving()){
                dualMotorLift.goToLevel(0);
                Log.v("DualMotorSlide- ChainBar", "gotoLevel(0) got called");
                lowerState=0;
            }
            telemetry.addLine("CB: chain bar is down, lowering slide");
            Log.v("ChainBar:", "chain bar is down, lowering slide");
        }
        SSKnocker.setPosition(ssKnockerPos);
        telemetry.addData("clawPos: ", claw.clawServo.getPosition());
        //telemetry.addData("slidePower:", slidePower);
        //telemetry.addData("target pos:", targetPosition);
        packet.put("time since cb movement", chainBar.lastMovement);
        telemetry.addData("is chainbar done?", chainBar.doneMoving());
        packet.put("is chainbar done?", chainBar.doneMoving());
        packet.put("slide target reached?", isLiftLevelReached());
        telemetry.update();

        // debug only,  remove it on release
        // packet.put("Current Position", slideMotor.getCurrentPosition());
        //  packet.put("target position", slideMotor.getTargetPosition());
        Log.v("DualMotorSlide-updatetarget", "ScoringSystem update returning.");
    }
}

