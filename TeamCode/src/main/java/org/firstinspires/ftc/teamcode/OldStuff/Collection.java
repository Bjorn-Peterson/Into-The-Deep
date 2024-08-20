package org.firstinspires.ftc.teamcode.OldStuff;

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

public class Collection {
    private ElapsedTime runtime = new ElapsedTime();
    Servo rotatorServo;
    Servo rotServo;
    Servo imAboutToDie;
    Servo imGoingToDie;
    Servo hangServo;
    Servo droneServo;
    Servo adjustServo;
    Servo angleServo;
    DcMotor hangMotor;
    DcMotor collectionMotor;
    NormalizedColorSensor colorSensor;
    NormalizedColorSensor delivSensor;
    private OpMode theOpMode;
    double armSpeed = 0;
    double imGoingToDie1;
    static final double imGoingToDie2 = .2;

    double imAboutToDie1;
    static final double imAboutToDie2 = .2;

    public Collection(HardwareMap hardwareMap, OpMode opMode) {
        theOpMode = opMode;
        rotatorServo = hardwareMap.get(Servo.class, "rotatorServo");
        rotServo = hardwareMap.get(Servo.class, "rotServo");
        imAboutToDie = hardwareMap.get(Servo.class, "leftBox");
        imGoingToDie = hardwareMap.get(Servo.class, "rightBox");
        hangMotor = hardwareMap.get(DcMotor.class, "hang");
        collectionMotor = hardwareMap.get(DcMotor.class, "collectionMotor");
        hangServo = hardwareMap.get(Servo.class, "hangServo");
        droneServo = hardwareMap.get(Servo.class, "droneServo");
        adjustServo = hardwareMap.get(Servo.class, "adjustServo");
        angleServo = hardwareMap.get(Servo.class, "angleServo");
        delivSensor = hardwareMap.get(NormalizedColorSensor.class, "delivSensor");

        collectionMotor.setDirection(DcMotor.Direction.FORWARD);

        rotatorServo.setDirection(Servo.Direction.FORWARD);
        imGoingToDie.setDirection(Servo.Direction.REVERSE);
        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        collectionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangServo.setDirection(Servo.Direction.FORWARD);
        droneServo.setDirection(Servo.Direction.REVERSE);
        imGoingToDie1 = imGoingToDie2;
        imAboutToDie1 = imAboutToDie2;
        //   colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
    }

    public void collectionTeleop() {
        /*
        if(theOpMode.gamepad2.y &&!theOpMode.gamepad2.x) {
            collectionMotor.setPower(-.4);
            collectionMotor.setTargetPosition(100);
            collectionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
        else if(!theOpMode.gamepad2.y && theOpMode.gamepad2.x)

        {
            collectionMotor.setPower(.4);
            collectionMotor.setTargetPosition(5);
        }
        else {
            collectionMotor.setTargetPosition(collectionMotor.getCurrentPosition());
        }

         */

    }

    public void setCollectionPosition() {

        if (theOpMode.gamepad2.dpad_left || theOpMode.gamepad1. dpad_left) {
            rotServo.setPosition(0);
        } else if (theOpMode.gamepad2.dpad_right || theOpMode.gamepad1.dpad_right) {
            rotServo.setPosition(0.22);
        }
        if (theOpMode.gamepad2.right_bumper) {
            adjustServo.setPosition(.2);
            imAboutToDie.setPosition(.78);
            imGoingToDie.setPosition(.78);
        }
        if (theOpMode.gamepad2.left_bumper) {
            adjustServo.setPosition(.8);

        }
        if (theOpMode.gamepad1.dpad_up) {
            imAboutToDie.setPosition(.86);
            imGoingToDie.setPosition(.86);
        } else if (theOpMode.gamepad1.dpad_down) {
            imAboutToDie.setPosition(.56);
            imGoingToDie.setPosition(.56);
        }
        collectionMotor.setPower(-theOpMode.gamepad2.right_stick_y * .5);
        if (theOpMode.gamepad1.x) {
            imAboutToDie.setPosition(1);
            imGoingToDie.setPosition(1);

        }
        if (theOpMode.gamepad1.ps) {
            rotServo.setPosition(.22);
        }

        if (theOpMode.gamepad1.right_bumper) {
            hangMotor.setPower(-.7);
        }
        else if (theOpMode.gamepad1.left_bumper) {
            hangMotor.setPower(.7);
        }
        else {
            hangMotor.setPower(theOpMode.gamepad2.left_stick_y * .7);
        }
        collectionMotor.setPower(-theOpMode.gamepad2.right_trigger);
        if (theOpMode.gamepad2.left_trigger > .1) {
            collectionMotor.setPower(theOpMode.gamepad2.left_trigger);
        }
        if (theOpMode.gamepad2.y) {
            rotatorServo.setPosition(.56);
        }
        else if (theOpMode.gamepad2.a) {
            rotatorServo.setPosition(.85);
        }
        else if (theOpMode.gamepad2.b) {
            rotatorServo.setPosition(0.01);
        }
        if (theOpMode.gamepad2.x) {
            hangServo.setPosition(0);
        }
        if (theOpMode.gamepad2.dpad_up) {
            droneServo.setPosition(.8);
        }
        if (theOpMode.gamepad2.dpad_down) {
            angleServo.setPosition(.17);
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

    }
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
}
