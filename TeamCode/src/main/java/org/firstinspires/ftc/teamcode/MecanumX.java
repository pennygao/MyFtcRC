package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.Math;

@Config
public class MecanumX {

    // ToDo 1: calibrate to make robot move straight (left/right balance, front/back balance, 2 direction balance)
    public static double LEFT_MOTOR_POWER_FACTOR = 0.96;
    public static double FRONT_MOTOR_POWER_FACTOR = 0.98;
    // front/back motors move robot to 45 degree direction, while the other 2 motors move robot to -45 degreee direction.
    // Adjust the factor to ensure robot can move at 0 degree direction.
    public static double AXIAL_MOTORS_POWER_FACTOR = 1.15;

    // ToDo 4: adjust k for auto-correct heading/drift
    public static double AUTO_CORRECT_HEADING_FACTOR = 0.02;
    public static double AUTO_CORRECT_DRIFT_FACTOR = 0.01;

    private final double ROBOT_MOVEMENT_CHANGE_LIMIT_PER_LOOP = 0.1;

    // Declare OpMode members for each of the 4 motors.
    private DcMotor leftFrontDrive = null;  // Front Motor
    private DcMotor leftBackDrive = null;   // Left Motor
    private DcMotor rightFrontDrive = null; // Right Motor
    private DcMotor rightBackDrive = null;  // Back Motor
    private boolean dryRun = false;

    private RobotLocalizer localizer = null;

    private Utilities utilities = null;

    private double axialPrevious = 0.0;
    private double lateralPrevious = 0.0;

    // reference for auto-correction
    private double referenceX;
    private double referenceY;
    private double referenceAngle;

    public MecanumX() {

        utilities = Utilities.getSharedUtility();

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = utilities.hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = utilities.hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = utilities.hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = utilities.hardwareMap.get(DcMotor.class, "right_back_drive");

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        localizer = new RobotLocalizer();
        localizer.initialize();
    }

    public void setMovement(double x_move, double y_move, double r_move, boolean dryRun, boolean auto_correct) {

        this.dryRun = dryRun;

        // For testing purpose, ensure robot does not move/rotate too fast
        x_move = x_move * 0.3;
        y_move = y_move * 0.3;
        double yaw = 0.2 * r_move;

        float currentOrientation = localizer.getAngle();
        double angleInRadians = Math.toRadians(currentOrientation);

        double adjustment = 0.0;

        if (auto_correct) {

            double changedX = localizer.getX() - referenceX;
            double changedY = localizer.getY() - referenceY;
            if (Math.abs(x_move) > 0 && Math.abs(y_move) > 0) {
                adjustment = AUTO_CORRECT_DRIFT_FACTOR * (Math.abs(changedX) -Math.abs(changedY));
                x_move -= Math.signum(changedX) * adjustment;
                Log.v("drivetrain/transition", String.format("Changed X/Y: %4.2f, %4.2f, diff: %4.2f, adjustment: %4.6f", changedX, changedY, Math.abs(changedX) -Math.abs(changedY), adjustment));
            } else if (Math.abs(x_move) > 0) {
                adjustment =  AUTO_CORRECT_DRIFT_FACTOR * changedY;
                y_move -= adjustment;
                Log.v("drivetrain/transition", String.format("Changed Y: %4.2f, adjustment: %4.6f", changedY, adjustment));
            } else if (Math.abs(y_move) > 0) {
                adjustment = AUTO_CORRECT_DRIFT_FACTOR * changedX;
                x_move -= adjustment;
                Log.v("drivetrain/transition", String.format("Changed X: %4.2f, adjustment: %4.6f", changedX, adjustment));
            }

            if (Math.abs(yaw) < 0.001) {
                // Driver is not requesting rotation
                double angleChanged = Utilities.getSharedUtility().angleMinus(localizer.getAngle(), referenceAngle);
                double yaw_adjustment = AUTO_CORRECT_HEADING_FACTOR * angleChanged;
                yaw += yaw_adjustment;
                Log.v("drivetrain/transition", String.format("Changed angle: %4.2f, adjustment: %4.6f", (localizer.getAngle() - referenceAngle), yaw_adjustment));
            }
        }

        // Convert to x/y according to the original orientation
        double axial = Math.cos(angleInRadians) * y_move - Math.sin(angleInRadians) * x_move;
        double lateral = Math.cos(angleInRadians) * x_move + Math.sin(angleInRadians) * y_move;
        if (Math.abs(axial) + Math.abs(lateral) > 0) {
            Log.v("drivetrain/transition", String.format("lateral/axial: %4.2f, %4.2f", lateral, axial));
        }

        // Make sure axial and lateral power does not change too fast
        if (Math.abs(axial - axialPrevious) > ROBOT_MOVEMENT_CHANGE_LIMIT_PER_LOOP) {
            axial += Math.signum(axial - axialPrevious) * ROBOT_MOVEMENT_CHANGE_LIMIT_PER_LOOP;
            axialPrevious = axial;
        }
        if (Math.abs(lateral - lateralPrevious) > ROBOT_MOVEMENT_CHANGE_LIMIT_PER_LOOP) {
            lateral += Math.signum(lateral - lateralPrevious) * ROBOT_MOVEMENT_CHANGE_LIMIT_PER_LOOP;
            lateralPrevious = lateral;
        }

//        utilities.telemetry.addData("Status", "Run Time: " + utilities.runtime.toString());
        utilities.telemetry.addData("Axial/Lateral/Yaw:", "%4.2f, %4.2f, %4.2f", axial, lateral, yaw);

        // Combine the movement requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;
        setMotorPower(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }

    private void setMotorPower(double leftFront, double rightFront, double leftBack, double rightBack) {

        double max;

        leftBack = leftBack * LEFT_MOTOR_POWER_FACTOR;
        leftFront = leftFront * FRONT_MOTOR_POWER_FACTOR * AXIAL_MOTORS_POWER_FACTOR;
        rightBack = rightBack * AXIAL_MOTORS_POWER_FACTOR;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFront), Math.abs(rightFront));
        max = Math.max(max, Math.abs(leftBack));
        max = Math.max(max, Math.abs(rightBack));

        if (max > 1.0) {
            leftFront  /= max;
            rightFront /= max;
            leftBack   /= max;
            rightBack  /= max;
        }

        if (!dryRun) {
            leftFrontDrive.setPower(leftFront);
            rightFrontDrive.setPower(rightFront);
            leftBackDrive.setPower(leftBack);
            rightBackDrive.setPower(rightBack);
        } else {
            leftFrontDrive.setPower(0.0);
            rightFrontDrive.setPower(0.0);
            leftBackDrive.setPower(0.0);
            rightBackDrive.setPower(0.0);
        }

        // Show the elapsed game time and wheel power.
        utilities.telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFront, rightFront);
        utilities.telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBack, rightBack);
        utilities.telemetry.addData("X/Y/Angle:", "%4.2f, %4.2f, %4.2f", localizer.getX(), localizer.getY(), localizer.getAngle());
        Utilities.getSharedUtility().telemetry.update();

        Telemetry dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();
        dashboardTelemetry.addData("x", "%4.2f", localizer.getX());
        dashboardTelemetry.addData("y", "%4.2f", localizer.getY());
        dashboardTelemetry.addData("heading", "%4.2f", localizer.getAngle());
        dashboardTelemetry.update();
    }

    public void saveReferencePosition() {
        referenceX = localizer.getX();
        referenceY = localizer.getY();
        Log.v("drivetrain/transition", "Saved current X and Y.");
    }

    public void saveReferenceAngle() {
        referenceAngle = localizer.getAngle();
        Log.v("drivetrain/transition", "Saved current Angle.");
    }
}

