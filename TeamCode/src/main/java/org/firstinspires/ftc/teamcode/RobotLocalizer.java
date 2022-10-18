package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Config
public class RobotLocalizer implements PeriodicUpdateCallback {

    public class OdometryWheel {
        private DcMotor encoder;
        private int currentTick;
        private int previousTick;
        private int tickPerRevolution;
        private double wheelRadius;

        public OdometryWheel(DcMotor encoder, int tickPerRevolution, double wheelRadius) {
            this.encoder = encoder;
            currentTick = encoder.getCurrentPosition();
            previousTick = currentTick;
            this.tickPerRevolution = tickPerRevolution;
            this.wheelRadius = wheelRadius;
        }

        public void update() {
            previousTick = currentTick;
            currentTick = encoder.getCurrentPosition();
        }

        public double distanceMoved(double adjustment) {
            double currentMovement = (double)currentTick / tickPerRevolution * 2 * Math.PI * wheelRadius;
            currentMovement -= adjustment;
            double previousMovement = (double)previousTick / tickPerRevolution * 2 * Math.PI * wheelRadius;
            return currentMovement - previousMovement;
        }

        public double totalDistanceMoved() {
            return ((double)currentTick) / tickPerRevolution * 2 * Math.PI * wheelRadius;
        }
    }

    // Now starts the defintion of class RobotLocalizer
    // ... ...

    // Use REV Through Bore Encoder, and 6cm diameter omni wheels
    private final int TICK_PER_REVOLUTION = 8192;
    private static float ODOMETRY_WHEEL_RADIUS = 30;

    // ToDo 2: Odometry wheels might not be perfectly perpendicular (or parallel to the heading) so need calibration.
    public static double AXIAL_WHEEL_ANGLE_FACTOR = 0.0;
    public static double LATERAL_WHEEL_ANGLE_FACTOR = 0.0;

    public static double LATERAL_WHEEL_DISTANCE_PER_DEGREE = 0.723;

    // ToDo 3: Adjust the odometry wheel radius in case it is slightly bigger/smaller
    public static double AXIAL_WHEEL_RADIUS_FACTOR = 1.0;
    public static double LATERAL_WHEEL_RADIUS_FACTOR = 1.0;

    // x, y, angle to be published. They are private to avoid being written accidentally and access should go through getters.
    private double x, y;
    private float angle;

    private BNO055IMU imu;
    private Orientation initialOrientation;
    private Orientation currentOrientation;
    private Orientation previousOrientation;

    private OdometryWheel axialWheel, lateralWheel;

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public float getAngle() {
        return angle;
    }

    public void initialize() {

        DcMotor encoder = Utilities.getSharedUtility().hardwareMap.get(DcMotor.class, "left_front_drive");
        axialWheel = new OdometryWheel(encoder, TICK_PER_REVOLUTION, AXIAL_WHEEL_RADIUS_FACTOR * ODOMETRY_WHEEL_RADIUS);
        encoder = Utilities.getSharedUtility().hardwareMap.get(DcMotor.class, "left_back_drive");
        lateralWheel = new OdometryWheel(encoder, TICK_PER_REVOLUTION, LATERAL_WHEEL_RADIUS_FACTOR * ODOMETRY_WHEEL_RADIUS);

        x = 0.0;
        y = 0.0;

        imu = Utilities.getSharedUtility().hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        imu.initialize(parameters);

        while (!imu.isGyroCalibrated()) {
            Utilities.sleep(10);
        }

        initialOrientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        previousOrientation = initialOrientation;
        Log.v("localizer/calibrate", "Initial raw angle: " + initialOrientation.firstAngle);

        Utilities.getSharedUtility().registerForPeriodicUpdate(this);
    }

    @Override
    public void update() {

        axialWheel.update();
        lateralWheel.update();

        currentOrientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        Log.v("localizer", "current raw angle: " + currentOrientation.firstAngle);
        angle = calculateAngleDelta(currentOrientation);
        double lateralWheelAdjustment = calculateAdjustment(currentOrientation, previousOrientation);
        previousOrientation = currentOrientation;

        double angleInRadians = Math.toRadians(angle);
        double displacementAxial = axialWheel.distanceMoved(0.0);
        // Lateral wheel moves while robot rotates. Need to deduct the movement caused by rotation.
        double displacementLateral = - lateralWheel.distanceMoved(lateralWheelAdjustment);
        Log.v("calibration", String.format("lateral movement adjusted: %4.2f", displacementLateral));
//        double displacementLateral = - lateralWheel.distanceMoved(0);

//        if (Math.abs(lateralWheel.distanceMoved()) > 0) {
//            Log.v("calibrate", String.format("Lateral wheel has moved %4.2fmm, rotated %4.2f degree.",
//                    lateralWheel.totalDistanceMoved(), totalAngle));
//        }

        // Laternal wheel mignt not be 90 degree to axial.
        displacementLateral += LATERAL_WHEEL_ANGLE_FACTOR * Math.abs(displacementAxial);
        displacementLateral += AXIAL_WHEEL_ANGLE_FACTOR * Math.abs(displacementAxial);

        if (Math.abs(displacementAxial) > 0.0 || Math.abs(displacementLateral) > 0.0) {
            // Increment after a coordinate system transformation (clockwise)
            x += displacementLateral * Math.cos(angleInRadians) - displacementAxial * Math.sin(angleInRadians);
            y += displacementLateral * Math.sin(angleInRadians) + displacementAxial * Math.cos(angleInRadians);
            Log.v("odometry", String.format("x/y adjusted: %4.2f, %4.2f", x, y));
        }
    }

    private float calculateAngleDelta(Orientation current) {

        // On my setup, orentitaiton angle increases (become positave) when robot rotates couterclockwise
        float angleDelta = current.firstAngle - initialOrientation.firstAngle;
        while (angleDelta > 180.0) {
            angleDelta -= 360.0;
        }
        while (angleDelta < -180.0) {
            angleDelta += 360.0;
        }
        Log.v("localizer", "current orientation: " + angleDelta);
        return angleDelta;
    }

    private double calculateAdjustment(Orientation current, Orientation previous) {

        double angleChanged = Utilities.getSharedUtility().angleMinus(current.firstAngle, previous.firstAngle);
        double adjustment = LATERAL_WHEEL_DISTANCE_PER_DEGREE * angleChanged;
        Log.v("calibrate", String.format("robot angle changed: %4.2f, adjustment: %4.2f", angleChanged, adjustment));
        return adjustment;
    }
}
