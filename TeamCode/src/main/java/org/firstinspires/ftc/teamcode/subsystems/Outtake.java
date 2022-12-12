package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Command;
import org.firstinspires.ftc.teamcode.robot.Subsystem;
import com.acmerobotics.roadrunner.util.NanoClock;


public class Outtake implements Subsystem {
    //Hardware: 1 motor, 1 encoder
    public DcMotorEx slideMotor;
    private double slidePower = 0.0;
    public static final double TICKS_PER_REV = 537.7*(11.0/15.0);
    public static final double PULLEY_DIAMETER = 38.0 / 25.4;
    public int level = 0;
    public static double SLIDE_LENGTH = 15.0;
    private static final double INCHES_PER_LEVEL = 15.5;
    public int targetPosition = 0;
    public Telemetry telemetry;
    private Servo obamaroller;
    private Servo SSKnocker;
    private double rollerPower = 0.5;
    boolean autoMode = false;
    Servo.Direction rollerDirection = Servo.Direction.FORWARD;

    private double[] level_ht = {5.0, 18.0, 29.0, 39.0, 32.0}; // in inches

    public class RollerCommand implements Command {
        NanoClock clock;
        double initialTimestamp;
        double seconds;
        double power;

        public RollerCommand(double power, double seconds) {
            this.power = power;
            this.seconds = seconds;
            clock = NanoClock.system();
            initialTimestamp = clock.seconds();
        }

        @Override
        public void start() {
            setRollerPower(power);
        }

        @Override
        public void update() {
        }

        @Override
        public void stop() {
            setRollerPower(0.5);
        }

        @Override
        public boolean isCompleted() {
            return clock.seconds() - initialTimestamp >= seconds;
        }
    }

    public RollerCommand rollerIntake(double power, double seconds) {
        return new RollerCommand(power, seconds);
    }

    public void setRollerPower(double power) {
        rollerPower = power;
    }

    public Outtake(Robot robot, boolean autoMode, Telemetry telemetry) {
        this.telemetry = telemetry;
        obamaroller = robot.getServo("obamaroller");
        slideMotor = robot.getMotor("slideMotor");
        SSKnocker = robot.getServo("SSKnocker");
        this.autoMode = autoMode;
        if (autoMode == true) {
            slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.setPower(1.0);
            telemetry.addLine("slideMotor in auto mode");
        }
        else{
            slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            telemetry.addLine("slideMotor not in auto mode");
        }
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.update();
        targetPosition = 0;

    }

    public void setSlideMotorMode (boolean encoderMode) {
        if (encoderMode == true) {
            slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.setPower(0.7);
            telemetry.addLine("slideMotor in auto mode");
        }
        else{
            slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            telemetry.addLine("slideMotor not in auto mode");
        }
    }

    public void setPower(double power) {
        slidePower = power;
    }

    public int inchToTicks(double inches) {
        return (int) (inches * TICKS_PER_REV / (PULLEY_DIAMETER * Math.PI));
    }

    public int getLevel() {
        return level;
    }

    public void goUp() {
        if (level < 3 && !slideMotor.isBusy()) {

            level = level + 1;
            targetPosition = inchToTicks(level_ht[level]);
            //targetPosition = inchToTicks (INCHES_PER_LEVEL * level);
            telemetry.addData("Slide level", level);
        }
    }

    public void goUp1Inch() {
        int current = slideMotor.getCurrentPosition();
        targetPosition = current + inchToTicks(1.5);
        /*
        if (level < 3 && !slideMotor.isBusy()) {
            targetPosition = inchToTicks(level_ht[level] + 1);
            telemetry.addData("Slide level", level);
        } */
    }

    public void goDown1Inch() {
        int current = slideMotor.getCurrentPosition();
        targetPosition = current - inchToTicks(1.5);
        /*
        if (level < 3 && !slideMotor.isBusy()) {
            targetPosition = inchToTicks(level_ht[level] - 1);
            telemetry.addData("Slide leve", level);
        }*/
    }

    public void goDown() {
        if (level > 0 && !slideMotor.isBusy()) { // slide_state.LEVEL_0) {
            level = level - 1;
            targetPosition = inchToTicks(level_ht[level]);
        }
    }

    public void goalldown() { // slide_state.LEVEL_0) {
            level = 0;
            targetPosition = inchToTicks(0);
    }
    public void goToIntake(){
        targetPosition = inchToTicks(4.42);
    }

    public void goToLevel(int level) {
        if (level <= 3 && !slideMotor.isBusy()) {

            this.level = level;
            targetPosition = inchToTicks(level_ht[level]);
            telemetry.addData("Slide leve", level);
        }
    }

    public void goToHt(double inches) {
        targetPosition = inchToTicks(inches);
        telemetry.addData("inches:",inches);
    }

    public boolean slideMotorBusy() {
        return slideMotor.isBusy();
    }


    @Override
    public void update(TelemetryPacket packet) {
        /*if (level == 0 &&  !slideMotor.isBusy()) {
            slidePower = 0;
        }
         */
        obamaroller.setPosition(rollerPower);
        //telemetry.addData("automode:",autoMode);
        //telemetry.addData("slidePower:", slidePower);
        //telemetry.addData("target pos:", targetPosition);
        //telemetry.update();
        if (slidePower != 0) {
            slideMotor.setPower(slidePower);

            if (this.autoMode) {
                slideMotor.setTargetPosition(targetPosition);
                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }
        // debug only,  remove it on release
        // packet.put("Current Position", slideMotor.getCurrentPosition());
        //  packet.put("target position", slideMotor.getTargetPosition());
    }
}

