package org.firstinspires.ftc.teamcode.Ri3d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OldStuff.Collection3d;
import org.firstinspires.ftc.teamcode.NewRobot.Lift;
import org.firstinspires.ftc.teamcode.OldStuff.PIDF;
import org.firstinspires.ftc.teamcode.OldStuff.ri3d;
import org.firstinspires.ftc.teamcode.OldStuff.OldLift;
@Autonomous
public class LeftSide extends LinearOpMode {

    public void runOpMode() {
        OldLift oldLift = new OldLift(hardwareMap, this, 145.1 , 1, 1.2);
        ri3d ri3d = new ri3d(hardwareMap, this, 384.5, 1, 4.09449);
        Collection3d collection = new Collection3d(hardwareMap, this);
        PIDF pidf = new PIDF(hardwareMap, this);


        waitForStart();

        ri3d.encoderDrive(.7, -11, 2);
        ri3d.strafeEncoderDrive(.5, -32, 3);
        pidf.auto(-150, 1);
        oldLift.liftAuto(.8, -1750, 3);
        oldLift.leftMotor.setPower(.4);
        oldLift.rightMotor.setPower(.4);
        ri3d.encoderDrive(.4, 4, 2);
        collection.rDelivery.setPosition(.8);
        sleep(1700);
        collection.rDelivery.setPosition(.4);
        collection.colState(Collection3d.collectionState.OPEN);
        oldLift.leftMotor.setPower(0);
        oldLift.rightMotor.setPower(0);
        ri3d.turnPID(10, 2);
        ri3d.encoderDrive(.4, -5,2);
        oldLift.liftAuto(-.8, -30, 2);
        sleep(800);
        collection.colState(Collection3d.collectionState.CLOSED);
        sleep(1000);
        collection.arm(.6, 2, 1.2);
        collection.colState(Collection3d.collectionState.OPEN);
        sleep(800);
        collection.arm(-.6, -100, 0.9);
        oldLift.liftAuto(.8, -1750, 4.4);
        oldLift.leftMotor.setPower(.4);
        oldLift.rightMotor.setPower(.4);
        ri3d.encoderDrive(.4, 4, 2);
        collection.rDelivery.setPosition(.8);
        sleep(1900);
        collection.rDelivery.setPosition(.4);
        oldLift.leftMotor.setPower(0);
        oldLift.rightMotor.setPower(0);
        oldLift.liftAuto(-.8, -30, 2);
        ri3d.turnPID(-20, 1);
        ri3d.encoderDrive(.4, -6, 2);


    }

}