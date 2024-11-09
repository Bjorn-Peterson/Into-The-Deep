package org.firstinspires.ftc.teamcode.OldStuff;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.DigitalChannel;

public class Collection3d {
    private ElapsedTime runtime = new ElapsedTime();
    Servo rCollection;
    Servo lCollection;
    Servo rDelivery;
    Servo lDelivery;
    Servo box;
    CRServo hangS;
    DcMotor collection;
    DcMotor hang;
    DigitalChannel cBeam;



    private OpMode theOpMode;

    public Collection3d(HardwareMap hardwareMap, OpMode opMode) {
        theOpMode = opMode;
        rCollection = hardwareMap.get(Servo.class, "rCollection");
        lCollection = hardwareMap.get(Servo.class, "lCollection");
        rDelivery = hardwareMap.get(Servo.class, "rDelivery");
        lDelivery = hardwareMap.get(Servo.class, "lDelivery");
        collection = hardwareMap.get(DcMotorEx.class, "collection");
        box = hardwareMap.get(Servo.class, "box");
        hang = hardwareMap.get(DcMotor.class, "hang");
        hangS = hardwareMap.get(CRServo.class, "hangS");

        rCollection.setDirection(Servo.Direction.FORWARD);
        lCollection.setDirection(Servo.Direction.FORWARD);
        collection.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rDelivery.setDirection(Servo.Direction.FORWARD);
        lDelivery.setDirection(Servo.Direction.REVERSE);

        cBeam = hardwareMap.get(DigitalChannel.class, "beam");

        cBeam.setMode(DigitalChannel.Mode.INPUT);

    }

    public void teleopControls() {

        if(theOpMode.gamepad1.dpad_up || theOpMode.gamepad2.dpad_up) {
        rDelivery.setPosition(.4);
    } else if(theOpMode.gamepad1.dpad_down || theOpMode.gamepad2.dpad_down) {
        rDelivery.setPosition(.7);
    }
        //Open
        if (theOpMode.gamepad1.left_bumper) {
            rCollection.setPosition(.71);
            lCollection.setPosition(.31);
        }
        //Close
        else if (theOpMode.gamepad1.right_bumper) {
            rCollection.setPosition(.61);
            lCollection.setPosition(.46);
        }
        /*
        if (theOpMode.gamepad1.x || theOpMode.gamepad2.x) {
            collection.setPower(.5);
        }
        else if (theOpMode.gamepad1.y || theOpMode.gamepad2.y) {
            collection.setPower(-.75);
        }
        else {
            collection.setPower(0);
        }

         */
        if (theOpMode.gamepad1.dpad_left || theOpMode.gamepad2.dpad_left) {
            box.setPosition(.1);
        }
        else if (theOpMode.gamepad1.dpad_right || theOpMode.gamepad2.dpad_right) {
            box.setPosition(.85);
        }
        if (theOpMode.gamepad2.a) {
            hang.setPower(.6);
        }
        else if (theOpMode.gamepad2.b) {
            hang.setPower(-.6);
        }
        else {
            hang.setPower(0);
        }

        if (theOpMode.gamepad2.left_bumper) {
            hangS.setPower(.9);
        }
        else if (theOpMode.gamepad2.right_bumper) {
            hangS.setPower(-.9);
        }
        else {
            hangS.setPower(0);
        }
        /*
        if (cBeam.getState() == false) {
            theOpMode.telemetry.addData("Beam", "Broken");
        } else {
            theOpMode.telemetry.addData("Beam", "NOT Broken");
        }

         */
}


public class CollectionAuto implements Action {
    private boolean initialized = false;
    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        if (!initialized) {
            collection.setPower(0.8);
            initialized = true;
        }

        double pos = collection.getCurrentPosition();
        packet.put("collectionPos", pos);
        if (pos < 3000.0) {
            return true;
        } else {
            collection.setPower(0);
            return false;
        }
    }
}



        // if (theOpMode.gamepad2.y) {
            /*
           rotatorServo.setPosition(.5);
            collectionMotor.setPower(.7);
            collectionMotor.setTargetPosition(150);
            collectionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rotatorServo.setPosition(.8);
            collectionMotor.setPower(-.7);
            collectionMotor.setTargetPosition(5);

             */


//        else {
//            collectionMotor.setTargetPosition(collectionMotor.getCurrentPosition());
//
//        }
//        if (theOpMode.gamepad2.x) {
//            collectionMotor.setTargetPosition(-150);
//            collectionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            collectionMotor.setPower(.4);
//        }






//}
/*

    public void openBox(double clawPos, double timeoutS) {
        runtime.reset();
        while (((LinearOpMode) theOpMode).opModeIsActive() && (runtime.seconds() < timeoutS)) {
            rotServo.setPosition(clawPos);
        }

    }
    public void moveClawStart(double clawPos, double timeoutS) {
        rotatorServo.setPosition(clawPos);
    }
    public boolean moveClawCheck(double clawPos, double timeoutS) {
        if (((LinearOpMode) theOpMode).opModeIsActive() && (runtime.seconds() < timeoutS)) {
            rotatorServo.setPosition(clawPos);
            return true;
        }
        else {
            moveClawEnd();
            return false;
        }
    }
    public void moveClawEnd() {

    }

    public void rotateArm(double armPos, double timeoutS) {
        imAboutToDie.setPosition(armPos);
        imGoingToDie.setPosition(armPos);

    }

    public void rotateArmStart(double armPos, double timeoutS) {
        imGoingToDie.setPosition(armPos);
        imAboutToDie.setPosition(armPos);
    }

    public boolean rotateArmCheck(double armPos, double timeoutS) {
        if (((LinearOpMode) theOpMode).opModeIsActive() && runtime.seconds() < timeoutS) {
            imGoingToDie.setPosition(armPos);
            imAboutToDie.setPosition(armPos);
            return true;
        }
        rotateArmEnd();
        return false;
    }

    public void rotateArmEnd() {
    }

    public void rotateClaw(double position, double timeoutS) {
        while (((LinearOpMode) theOpMode).opModeIsActive() && runtime.seconds() < timeoutS) {
            rotServo.setPosition(position);
        }
    }

    public void rotateClawStart(double position, double timeoutS) {
        rotServo.setPosition(position);
    }

    public boolean rotateClawCheck(double position, double timeoutS) {
        if (((LinearOpMode) theOpMode).opModeIsActive() && runtime.seconds() < timeoutS) {
            rotServo.setPosition(position);
            return true;
        }
        rotateClawEnd();
        return false;
    }

    public void rotateClawEnd() {
    }

    public void collectionArm(int rotation, double power, double timeoutS) {
        runtime.reset();
        collectionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        collectionMotor.setTargetPosition(rotation);
        collectionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        collectionMotor.setPower(Math.abs(power));

        while (((LinearOpMode) theOpMode).opModeIsActive() && runtime.seconds() < timeoutS && collectionMotor.isBusy()) {
            theOpMode.telemetry.addData("targetPosition", 0);
            theOpMode.telemetry.addData("CurrentPosition", 0);

        }
        collectionMotor.setPower(0);
        collectionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void collectionArmStart(int rotation, double power, double timeoutS) {
        collectionMotor.setPower(power);
        collectionMotor.setTargetPosition(rotation);
        collectionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        collectionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //something or rather
    }

    public boolean collectionArmCheck(int rotation, double power, double timeoutS) {
        if (((LinearOpMode) theOpMode).opModeIsActive() && runtime.seconds() < timeoutS && collectionMotor.isBusy()) {
            return true;
        }
        collectionArmEnd();
        return false;
    }
    public void collectionArmEnd() {
        collectionMotor.setPower(0);
    }
    public void collection(double power, double rotations, double timeoutS) {
        runtime.reset();
        hangMotor.setPower(power);
        while (((LinearOpMode) theOpMode).opModeIsActive() && runtime.seconds() < timeoutS && collectionMotor.isBusy()) {

        }
        hangMotor.setPower(0);
    }
    public void collectionStart(double power, double rotations, double timeoutS) {
        runtime.reset();
        hangMotor.setPower(power);
    }
    public boolean collectionCheck(double power, double rotations, double timeoutS) {
        if (((LinearOpMode) theOpMode).opModeIsActive() && runtime.seconds() < timeoutS) {
            return true;
        }
        else {
            collectionEnd();
            return false;
        }
    }
    public void collectionEnd() {
        hangMotor.setPower(0);
    }

 */
}
