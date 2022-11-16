package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
//import org.firstinspires.ftc.robotcore.external.tfod.TfodCurrentGame;
import org.firstinspires.ftc.teamcode.robot.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.TfodSS;

@TeleOp(name = "tensorFlow (Blocks to Java)")
public class tensorFlow extends LinearOpMode {

    private VuforiaCurrentGame vuforiaFreightFrenzy;
    //private TfodCurrentGame tfodFreightFrenzy;
    private TfodSS tfodFreightFrenzy;

    Recognition recognition;
    boolean isDuckDetected;
    int     imgIdx = 0;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        List<Recognition> recognitions;
        int index;

        vuforiaFreightFrenzy = new VuforiaCurrentGame();
        tfodFreightFrenzy = new TfodSS();

        // Sample TFOD Op Mode
        // Initialize Vuforia.
        // This sample assumes phone is in landscape mode.
        // Rotate phone -90 so back camera faces "forward" direction on robot.
        // We need Vuforia to provide TFOD with camera images.
        vuforiaFreightFrenzy.initialize(
                "", // vuforiaLicenseKey
                VuforiaLocalizer.CameraDirection.BACK, // cameraDirection
                false, // useExtendedTracking
                false, // enableCameraMonitoring
                VuforiaLocalizer.Parameters.CameraMonitorFeedback.NONE, // cameraMonitorFeedback
                0, // dx
                0, // dy
                0, // dz
                AxesOrder.XYZ, // axesOrder
                0, // firstAngle
                -90, // secondAngle
                0, // thirdAngle
                true); // useCompetitionFieldTargetLocations
        // Set min confidence threshold to 0.7
        tfodFreightFrenzy.initialize(vuforiaFreightFrenzy, (float) 0.5, true, true);
        // Initialize TFOD before waitForStart.
        // Init TFOD here so the object detection labels are visible
        // in the Camera Stream preview window on the Driver Station.
        tfodFreightFrenzy.activate();
        // Enable following block to zoom in on target.
        tfodFreightFrenzy.setZoom(2.5, 16 / 9);
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        // Wait for start command from Driver Station.
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            int cnt = 0;
            while (opModeIsActive()) {
                // Put loop blocks here.
                // Get a list of recognitions from TFOD.

                // If list is empty, inform the user. Otherwise, go
                // through list and display info for each recognition.
                for (int i=0; i< 10; i++) {
                    recognitions = tfodFreightFrenzy.getRecognitions();
                    if (recognitions.size() == 0) {
                        imgIdx = 0;
                        telemetry.addData("TFOD", "No items detected.");
                    } else {
                        String max_lab = "";
                        double max = 0;
                        index = 0;

                        // Iterate through list and call a function to
                        // display info for each recognized object.
                        for (Recognition recognition_item : recognitions) {
                            recognition = recognition_item;
                            // Display info.
                            displayInfo(cnt, index);
                            // Increment index.
                            index = index + 1;

                            double left = Double.parseDouble(JavaUtil.formatNumber(recognition.getLeft(), 0));
                            double right = Double.parseDouble(JavaUtil.formatNumber(recognition.getRight(), 0));
                            double top = Double.parseDouble(JavaUtil.formatNumber(recognition.getTop(), 0));
                            double bottom = Double.parseDouble(JavaUtil.formatNumber(recognition.getBottom(), 0));
                            double sz = Math.abs(top - bottom) * Math.abs(right - left);
                            if (sz > max) {
                                max = sz;
                                max_lab = recognition.getLabel();
                            }
                        }
                        telemetry.addData("TFOD found image: ", max_lab);
                        telemetry.addData("TFOD found image sz: ", max);
                        telemetry.update();
                        break;

                    }// else
                    cnt++;
                }
                telemetry.update();
            }
        }
        // Deactivate TFOD.
        tfodFreightFrenzy.deactivate();

        vuforiaFreightFrenzy.close();
        tfodFreightFrenzy.close();
    }

/**
 * Display info (using telemetry) for a recognized object.
 */
private void displayInfo(int cnt, int i) {
    // Display label info.
    // Display the label and index number for the recognition.
    telemetry.addData("cnt:  ", cnt);
    telemetry.addData("label " + i, recognition.getLabel());
    // Display upper corner info.
    // Display the location of the top left corner
    // of the detection boundary for the recognition
    telemetry.addData("Left, Top " + i, Double.parseDouble(JavaUtil.formatNumber(recognition.getLeft(), 0)) + ", " + Double.parseDouble(JavaUtil.formatNumber(recognition.getTop(), 0)));
    // Display lower corner info.
    // Display the location of the bottom right corner
    // of the detection boundary for the recognition
    telemetry.addData("Right, Bottom " + i, Double.parseDouble(JavaUtil.formatNumber(recognition.getRight(), 0)) + ", " + Double.parseDouble(JavaUtil.formatNumber(recognition.getBottom(), 0)));
    telemetry.addData("confidence: ", recognition.getConfidence());
    // Display Recognition of Duck or Team Element

}
}