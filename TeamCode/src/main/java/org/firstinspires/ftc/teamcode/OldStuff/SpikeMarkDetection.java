package org.firstinspires.ftc.teamcode.OldStuff;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.ArrayList;

public class SpikeMarkDetection {
    public enum spikeMarkPositions {
        LEFT,
        MIDDLE,
        RIGHT,
        NOTHING
    }
    String something;
    private volatile SpikeMarkDetection.spikeMarkPositions positions;
    OpenCvCamera webcam;
    SkystoneDeterminationPipeline pipeline;
    private LinearOpMode theOpMode;

    public SpikeMarkDetection(HardwareMap hardwareMap, LinearOpMode opMode) {
        theOpMode = opMode;
    }


    public spikeMarkPositions detectPosition(boolean isRed) {
        spikeMarkPositions positions = spikeMarkPositions.NOTHING;

        int cameraMonitorViewId = theOpMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", theOpMode.hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(theOpMode.hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new SkystoneDeterminationPipeline();
        pipeline.isRed = isRed;
        webcam.setPipeline(pipeline);


        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
        while (theOpMode.opModeInInit()) {
            if (pipeline.square1Activated) {
                positions = SpikeMarkDetection.spikeMarkPositions.MIDDLE;
                theOpMode.telemetry.addData("Position 1?","");
                theOpMode.telemetry.update();

            }
            else if (pipeline.square2Activated) {
                positions = SpikeMarkDetection.spikeMarkPositions.RIGHT;
                theOpMode.telemetry.addData("Position 2?","");
                theOpMode.telemetry.update();
            }
            else {
                positions = SpikeMarkDetection.spikeMarkPositions.LEFT;
                theOpMode.telemetry.addData("Position 3?","");
                theOpMode.telemetry.update();
            }
        }
        return positions;
    }


}
