
package org.firstinspires.ftc.teamcode.OldStuff;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;

@Autonomous
public class RedLeft extends LinearOpMode {

    public void runOpMode() {

        Lift lift = new Lift(hardwareMap, this, 537.5, 1, 2);
        ri3d ri3d = new ri3d(hardwareMap, this, 384.5, 1, 4.0);

        waitForStart();
    }
}
        /*driveTrain.encoderDrive(.2,-22,3);
        driveTrain.encoderDrive(.2,4,2);
        sleep(250);
        driveTrain.turnToPID(45,2);
        driveTrain.encoderDrive(.1,-12,2);
        driveTrain.turnToPID(0,2);
        driveTrain.encoderDrive(.1,-13,2);
        driveTrain.turnToPID(-270,2);
        driveTrain.encoderDrive(.25,67,5);
        driveTrain.turnToPID(0,2);
        driveTrain.encoderDrive(.1,22,3);
        driveTrain.turnToPID(90,2);
        driveTrain.encoderDrive(.1,0.1,2);
        driveTrain.driveToDistanceSensor(.25,2,4);
        liftBox(.8, 500, .84, 2, lift, collection);
        collection.rotServo.setPosition(0);
        sleep(250);
        liftBox(-.3, -300, .55, 2, lift, collection);
        driveTrain.encoderDrive(.1,-4,2);
        sleep(250);
        liftBox(-.3, -300, .55, 2, lift, collection);*/


        //the middle detection was being extremely weird.
        // it was going the same distance on both sides, but both sides had different distance inputs
/*
        if (position == (SpikeMarkDetection.spikeMarkPositions.LEFT)) {
            driveTrain.awDrive(.1, .85, .1, .85, -13, 2);
            sleep(1000);
            driveTrain.encoderDrive(.1,-5,2);
            driveTrain.encoderDrive(.1,13.5,3);
            driveTrain.turnToPID(0,2);
            driveTrain.encoderDrive(.2,-26.5,2);
            driveTrain.turnToPID(270,2);
            driveTrain.encoderDrive(.25,-45,2);
            driveTrain.turnToPID(0,2);
            driveTrain.encoderDrive(.1,18,2);
            driveTrain.turnToPID(88,2);
            driveTrain.encoderDrive(.1,0.1,2);
            driveTrain.driveToDistanceSensor(.25,3,4);
            liftBox(.8, 500, .84, 2, lift, collection);
            collection.rotServo.setPosition(0);
            sleep(250);
            liftBox(-.3, -300, .55, 2, lift, collection);
            driveTrain.encoderDrive(.1,-4,2);
            sleep(250);
            liftBox(-.3, -300, .55, 2, lift, collection);
        }
        else if (position == (SpikeMarkDetection.spikeMarkPositions.MIDDLE)) {
            driveTrain.encoderDrive(.2,-22,3);
            driveTrain.encoderDrive(.2,4,2);
            sleep(250);
            driveTrain.turnToPID(45,2);
            driveTrain.encoderDrive(.1,-12,2);
            driveTrain.turnToPID(0,2);
            driveTrain.encoderDrive(.1,-13,2);
            driveTrain.turnToPID(-270,2);
            driveTrain.encoderDrive(.25,70,5);
            driveTrain.turnToPID(0,2);
            driveTrain.encoderDrive(.1,14.5,2);
            driveTrain.turnToPID(90,2);
            driveTrain.encoderDrive(.1,0.1,2);
            driveTrain.driveToDistanceSensor(.25,2,4);
            liftBox(.8, 500, .84, 2, lift, collection);
            collection.rotServo.setPosition(0);
            sleep(250);
            liftBox(-.3, -300, .55, 2, lift, collection);
            driveTrain.encoderDrive(.1,-4,2);
            sleep(250);
            liftBox(-.3, -300, .55, 2, lift, collection);
        }
        if (position == (SpikeMarkDetection.spikeMarkPositions.RIGHT)) {
            driveTrain.encoderDrive(.1,-10,3);
            driveTrain.turnToPID(-45,2);
            driveTrain.encoderDrive(.1,-8,2);
            driveTrain.encoderDrive(.1,6,2);
            driveTrain.turnToPID(0,2);
            driveTrain.encoderDrive(.1,-16.5,3);
            driveTrain.turnToPID(270,2);
            driveTrain.encoderDrive(.25,-45,2);
            driveTrain.turnToPID(0,2);
            driveTrain.encoderDrive(.1,26.5,2);
            driveTrain.turnToPID(88,2);
            driveTrain.encoderDrive(.1,0.1,2);
            driveTrain.driveToDistanceSensor(.25,3,4);
            liftBox(.8, 500, .84, 2, lift, collection);
            collection.rotServo.setPosition(0);
            sleep(250);
            liftBox(-.3, -300, .55, 2, lift, collection);
            driveTrain.encoderDrive(.1,-4,2);
            sleep(250);
            liftBox(-.3, -300, .55, 2, lift, collection);
        }

    }



    public void liftBox(double power, int target, double armPos, double timeoutS, Lift givenLift, Collection givenCollection) {
        givenLift.liftAutoStart(power, target, timeoutS);
        givenCollection.rotateArmStart(armPos, timeoutS);
        while (opModeIsActive() && givenLift.liftAutoCheck(power, target, timeoutS) || givenCollection.rotateArmCheck(armPos, timeoutS));
    }
    public void driveRotate(double speed, double inches, double clawPos, double timeoutS, Drivetrain givenDrivetrain, Collection givenCollection) {
        givenDrivetrain.encoderDriveStart(speed, inches, timeoutS);
        givenCollection.moveClawStart(clawPos, timeoutS);
        while (opModeIsActive() && givenDrivetrain.encoderDriveCheck(speed, inches, timeoutS) || givenCollection.moveClawCheck(clawPos, timeoutS));
    }
    public void driveRotateSpin(double speed, double inches, double clawPos, double power, double rotations, double timeoutS, Drivetrain givenDrivetrain, Collection givenCollection) {
        givenDrivetrain.encoderDriveStart(speed, inches, timeoutS);
        givenCollection.moveClawStart(clawPos, timeoutS);
        givenCollection.collectionStart(power, rotations, timeoutS);
        while (opModeIsActive() && givenDrivetrain.encoderDriveCheck(speed, inches, timeoutS) || givenCollection.moveClawCheck(clawPos, timeoutS) || givenCollection.collectionCheck(power, rotations, timeoutS));
 */
