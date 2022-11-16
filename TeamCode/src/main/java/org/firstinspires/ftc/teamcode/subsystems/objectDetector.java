package org.firstinspires.ftc.teamcode.subsystems;
import android.util.Log;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TfodCurrentGame;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Subsystem;


import java.util.List;


public class objectDetector implements Subsystem {
    public static int    NUM_TRY = 20;
    public static double CONFIDENCE = 0.5;


    private VuforiaCurrentGame vuforiaFreightFrenzy;
    //private TfodCurrentGame tfodFreightFrenzy;
    private TfodSS tfodPowerPlay;
    Recognition recognition;
    Telemetry telemetry;
    Robot     robot;

    public objectDetector(Robot robot, Telemetry telemetry) {
        vuforiaFreightFrenzy = new VuforiaCurrentGame();
        //tfodFreightFrenzy = new TfodCurrentGame();
        tfodPowerPlay = new TfodSS();
        this.telemetry = telemetry;
        this.robot = robot;
    }

    public void init() {
        vuforiaFreightFrenzy.initialize(
                "", // vuforiaLicenseKey
                VuforiaLocalizer.CameraDirection.BACK, // cameraDirection
                false, // useExtendedTracking
                false, // enableCameraMonitoring
                VuforiaLocalizer.Parameters.CameraMonitorFeedback.NONE, // cameraMonitorFeedback
                0, // dx
                0, //dy
                0, // dz
                AxesOrder.XYZ, // axesOrder
                0, // firstAngle
                -90, // secondAngle
                0, // thirdAngle
                true); // useCompetitionFieldTargetLocations
        // TODO: Set min confidence threshold to 0.7
        tfodPowerPlay.initialize(vuforiaFreightFrenzy, (float)CONFIDENCE, true, true);
        // Initialize TFOD before waitForStart.
        // Init TFOD here so the object detection labels are visible
        // in the Camera Stream preview window on the Driver Station.
        tfodPowerPlay.activate();
        // Enable following block to zoom in on target.
        tfodPowerPlay.setZoom(2.5, 16 / 9);
    }

    public void close() {
        tfodPowerPlay.deactivate();
        vuforiaFreightFrenzy.close();
        tfodPowerPlay.close();
    }



    private void displayInfo (int i, Recognition recognition){
        // Display label info.
        // Display the label and index number for the recognition.
        Log.i("Index", "%0d" + i);
        Log.i("Label %s", recognition.getLabel());
        // Display upper corner info.
        // Display the location of the top left corner
        // of the detection boundary for the recognition
        Log.i("Left, Top ", Double.parseDouble(JavaUtil.formatNumber(recognition.getLeft(), 0)) + ", " + Double.parseDouble(JavaUtil.formatNumber(recognition.getTop(), 0)));
        // Display lower corner info.
        // Display the location of the bottom right corner
        // of the detection boundary for the recognition
        Log.i("Right, Bottom ", Double.parseDouble(JavaUtil.formatNumber(recognition.getRight(), 0)) + ", " + Double.parseDouble(JavaUtil.formatNumber(recognition.getBottom(), 0)));
        telemetry.addData("Object  detected: ", recognition.getLabel());
        telemetry.addData("Confidence: ", recognition.getConfidence());
    }



    // if Lable is Duck, and left < threshold, pos = 1
    // if Label is Duck, and left >= threshold pos = 2
    // if Label is not Duck, or nothing, pos = 3
    // recognitions.size() == 0
    // recognition.getLabel().equals("Duck")
    // recognition.getLeft()
    public int ssIndex(int num_try) { // 1: img1, 2: img2, 3: img3, 4 : none
        //int[] pos_cnt = {0,0,0};
        int pos = 4;
        List<Recognition> recognitions;
        int index;

        for (int i=0; i< num_try; i++) {
            recognitions = tfodPowerPlay.getRecognitions();
            if (recognitions.size() == 0) {
                telemetry.addData("TFOD", "No items detected.");
                telemetry.update();
            } else {
                String max_lab = "";
                double max = 0;
                index = 0;

                // Iterate through list and call a function to
                // display info for each recognized object.
                for (Recognition recognition_item : recognitions) {
                    recognition = recognition_item;
                    // Display info.
                    displayInfo(index, recognition);
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

                if(max_lab.equals("img1")){
                    pos = 1;
                }
                else if (max_lab.equals("img2")){
                    pos = 2;
                }
                else if (max_lab.equals("img3")){
                    pos = 3;
                }
                telemetry.addData(" TFOD found image = ", max_lab);
                telemetry.update();
                break;
            }
            //sleep(10);
        }
/*
        int pos = 4;
        int max_cnt = 0;
        for (int i=0; i< 3; i++) {
            if (pos_cnt[i] > max_cnt) {
                pos = i;
                max_cnt = pos_cnt[i];
            }
        }

 */
        //telemetry.addData("pos = ",pos);
        //telemetry.update();

        return (pos);
    }


    @Override
    public void update(TelemetryPacket packet) {

    }
}
