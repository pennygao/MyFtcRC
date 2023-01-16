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
    public DualMotorLift dualMotorLift;
    public Telemetry telemetry;

    public int swingState = 0;
    private int lowerState = 0;

    private Servo SSKnocker;
    private double ssKnockerPos = 0;

    public ChainBar chainBar;
    public Claw claw;
    //TODO: tune
    //public static double CLAW_OPEN_POSITION = 0.38;
    //public static double CLAW_CLOSE_POSITION = 0.58; //0
    public static double CLAW_OPEN_POSITION = 0.48;
    public static double CLAW_CLOSE_POSITION = 0.78;


    public static double CHAIN_BAR_DOWN = 0.44;
    public static double CHAIN_BAR_UP = 0.14; //How much you move the chain bar up from down position

    public class Claw {
        public Servo clawServo;
        NanoClock clock;
        double initialTimestamp;
        double movementTime;
        double toPos;

        private double openPos;
        private double closePos;

        public Claw(Robot robot, double openPos, double closePos){
            clawServo = robot.getServo("claw");
            this.openPos=openPos;
            this.closePos=closePos;
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

        public void openClaw(){
            claw.setClawPos(CLAW_OPEN_POSITION);
        }

        public void closeClaw(){
            claw.setClawPos(CLAW_CLOSE_POSITION);
        }
    }

    public class ChainBar{
        public Servo cbServo;
        private double lastMovement=0;
        //TODO: tune these values to actual chain bar
        private int CBDirection = 0;

        public ChainBar(Robot robot){
            cbServo = robot.getServo("chainBar");
        }

        public void swing(){
            cbServo.setPosition(CHAIN_BAR_DOWN + CHAIN_BAR_UP*CBDirection);
            lastMovement = NanoClock.system().seconds();
        }

        public void swing_direction(int direction){
            cbServo.setPosition(CHAIN_BAR_DOWN + CHAIN_BAR_UP * direction);
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
            return (NanoClock.system().seconds() - lastMovement > 2);
        }
        public void adjust(int direction){
            double toPosition = cbServo.getPosition(); //get target position
            cbServo.setPosition(toPosition+(direction*0.00625));
        }
    }

    public ScoringSystem(Robot robot, boolean autoMode, Telemetry telemetry) {
        this.telemetry = telemetry;
        claw = new Claw(robot, CLAW_OPEN_POSITION, CLAW_CLOSE_POSITION);
        chainBar = new ChainBar(robot);
        SSKnocker = robot.getServo("SSKnocker");
        dualMotorLift = new DualMotorLift(robot, telemetry, DualMotorLift.Mode.BOTH_MOTORS_PID);
        //dualMotorLift = new DualMotorLift(robot, telemetry, DualMotorLift.Mode.RIGHT_FOLLOW_LEFT);
        telemetry.update();

    }

    public void adjustLift(int direction, boolean slow){
        dualMotorLift.adjustLift(direction, slow);
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


    @Override
    public void update(TelemetryPacket packet) {
        dualMotorLift.update(packet);
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
        //telemetry.addData("clawPos: ", claw.clawServo.getPosition());
        //telemetry.addData("slidePower:", slidePower);
        //telemetry.addData("target pos:", targetPosition);
        packet.put("time since cb movement", chainBar.lastMovement);
        //telemetry.addData("is chainbar done?", chainBar.doneMoving());
        packet.put("is chain bar done?", chainBar.doneMoving());
        //telemetry.addData("slide target reached?", isLiftLevelReached());
        //telemetry.addData("Chain Bar position: ", chainBar.cbServo.getPosition());
        //telemetry.update();

        // debug only,  remove it on release
        // packet.put("Current Position", slideMotor.getCurrentPosition());
        //  packet.put("target position", slideMotor.getTargetPosition());
        Log.v("DualMotorSlide-updatetarget", "ScoringSystem update returning.");
    }
}

