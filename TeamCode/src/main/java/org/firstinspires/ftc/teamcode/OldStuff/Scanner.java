package org.firstinspires.ftc.teamcode.OldStuff;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

public class Scanner {
    OpenCvWebcam webcam;
    private LinearOpMode theOpMode;
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family
    AprilTagDetection tagOfInterest = null;

    public Scanner(HardwareMap hardwareMap, LinearOpMode opMode) {
        theOpMode = opMode;

    }

    public int scanSignal() {
        String signalPosition;
        int aprilID = 0;

        int cameraMonitorViewId = theOpMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id",
                theOpMode.hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(theOpMode.hardwareMap.get(WebcamName.class, "Webcam 1"),
                cameraMonitorViewId);
        AprilTagDetectionPipeline aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        webcam.setPipeline(aprilTagDetectionPipeline);
        //   SkystoneDeterminationPipeline skystoneDeterminationPipeline = new SkystoneDeterminationPipeline(tagsize, fx, fy, cx, cy);

        //    webcam.setPipeline(SkystoneDeterminationPipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);

            }

            @Override
            public void onError(int errorCode) {
            }
        });
        signalPosition = "";
        int counter = 0;
        while ((theOpMode.opModeInInit())) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
            currentDetections.size();
            theOpMode.telemetry.addData("April ID", aprilID);
            theOpMode.telemetry.update();
            if (currentDetections.size() > 0) {
                aprilID = currentDetections.get(0).id;
            }

        }
        return aprilID;

    }
}
