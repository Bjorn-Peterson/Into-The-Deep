package org.firstinspires.ftc.teamcode.Ri3d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OldStuff.Collection3d;
import org.firstinspires.ftc.teamcode.OldStuff.ri3d;
import org.firstinspires.ftc.teamcode.OldStuff.OldLift;
@Autonomous
public class RightAuto extends LinearOpMode {

    public void runOpMode() {
        OldLift oldLift = new OldLift(hardwareMap, this, 145.1 , 1, 1.2);
        ri3d ri3d = new ri3d(hardwareMap, this, 384.5, 1, 4);
        Collection3d collection = new Collection3d(hardwareMap, this);


        waitForStart();

        ri3d.encoderDrive(.7, -9.5, 2);
        ri3d.strafeEncoderDrive(.4, -32, 3);
        collection.collection.setPower(-.6);
        sleep(300);
        collection.collection.setPower(0);
        oldLift.liftStart = true;
        //Deliver First
        oldLift.auto();
        oldLift.leftMotor.setPower(-.1);
        ri3d.encoderDrive(.7, -1.2, 2);
        //Collect First
        collection.colState(Collection3d.collectionState.CLOSED);
        sleep(800);
        collection.collection.setPower(.65);
        sleep(300);
        collection.collection.setPower(0);
        sleep(500);
        collection.colState(Collection3d.collectionState.OPEN);
        ri3d.encoderDrive(.6, 1, 2);
        collection.collection.setPower(-.6);
        sleep(300);
        collection.collection.setPower(0);
        oldLift.liftStart = true;
        //Deliver Second
        oldLift.auto();
        ri3d.turnToPID(-15, 1);
        ri3d.encoderDrive(.4, -3.2, 2);
        //Collect Second
        collection.colState(Collection3d.collectionState.CLOSED);
        sleep(800);
        collection.collection.setPower(.65);
        sleep(300);
        collection.collection.setPower(0);
        sleep(500);
        collection.colState(Collection3d.collectionState.OPEN);
        ri3d.strafeEncoderDrive(.4, 6, 2);
        collection.collection.setPower(-.6);
        sleep(300);
        collection.collection.setPower(0);
        ri3d.encoderDrive(.4, 5, 2);
        oldLift.liftStart = true;
        //Deliver 3rd
        oldLift.auto();
        ri3d.turnToPID(-27, 1);
        ri3d.encoderDrive(.6, -4.5, 2);
        collection.colState(Collection3d.collectionState.CLOSED);
        sleep(800);
        collection.collection.setPower(.65);
        sleep(300);
        collection.collection.setPower(0);
        sleep(500);
        collection.colState(Collection3d.collectionState.OPEN);
        ri3d.encoderDrive(.6, 6, 2);
        collection.collection.setPower(-.6);
        sleep(300);
        collection.collection.setPower(0);
        oldLift.liftStart = true;
        oldLift.auto();
        ri3d.encoderDrive(.8, -42, 3);
        ri3d.turnToPID(-90, 1);
        collection.collection.setPower(.65);
        sleep(300);
        collection.collection.setPower(0);
        ri3d.encoderDrive(.8, -12, 2);
        collection.collection.setPower(-.7);
        sleep(200);
        collection.collection.setPower(0);


    }

}