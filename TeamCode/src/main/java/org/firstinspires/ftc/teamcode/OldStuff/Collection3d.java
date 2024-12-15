package org.firstinspires.ftc.teamcode.OldStuff;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Collection3d {

    public enum collectionState {
        CLOSED,
        OPEN,
        WIDE
    }

    public enum AutoState {
        START,
        COLLECT,
        DELIVER,
        RETURN
    }

    AutoState autoState = AutoState.START;
    private ElapsedTime runtime = new ElapsedTime();
    public Servo rCollection;
    public Servo lCollection;
    public Servo rDelivery;
    public Servo lDelivery;
    Servo box;
    CRServo hangS;
    public DcMotorEx collection;
    DcMotor hang;
    DigitalChannel cBeam;

    public static double p = 0.005, i = 0, d = 0.001;
    public static double f = 0.1;
    public static int target;
    private final double ticksPerInch = 100;
    ElapsedTime collectionTimer = new ElapsedTime();
    ElapsedTime transferTimer = new ElapsedTime();
    public boolean collectionStart;
    public boolean collectionDone;

    private PIDController controller;
    private OpMode theOpMode;
    double rOpen = .6;
    double rClosed = .71;
    double lOpen = .7;
    double lClosed = .631


            ;

    public Collection3d(HardwareMap hardwareMap, OpMode opMode) {
        controller = new PIDController(p, i, d);
        theOpMode = opMode;
        rCollection = hardwareMap.get(Servo.class, "rCollection");
        lCollection = hardwareMap.get(Servo.class, "lCollection");
        rDelivery = hardwareMap.get(Servo.class, "rDelivery");
        lDelivery = hardwareMap.get(Servo.class, "lDelivery");
        collection = hardwareMap.get(DcMotorEx.class, "collection");
        box = hardwareMap.get(Servo.class, "box");
        hang = hardwareMap.get(DcMotor.class, "hang");
        hangS = hardwareMap.get(CRServo.class, "hangS");
        collection.setDirection(DcMotorSimple.Direction.REVERSE);
        collection.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        collection.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        collection.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        collection.setCurrentAlert(4.5, CurrentUnit.AMPS);

        lDelivery.setDirection(Servo.Direction.REVERSE);

        cBeam = hardwareMap.get(DigitalChannel.class, "beam");

        cBeam.setMode(DigitalChannel.Mode.INPUT);

    }

    public void teleopControls() {
        //Deliver
        if (theOpMode.gamepad1.dpad_up || theOpMode.gamepad2.dpad_up) {
            rDelivery.setPosition(.4);
        }
        //Return
        else if (theOpMode.gamepad1.dpad_down || theOpMode.gamepad2.dpad_down) {
            rDelivery.setPosition(.8);
        }
        //Open
        if (theOpMode.gamepad1.left_bumper) {
            rCollection.setPosition(lOpen);
            lCollection.setPosition(rOpen);
        }
        //Close
        else if (theOpMode.gamepad1.right_bumper) {
            rCollection.setPosition(lClosed);
            lCollection.setPosition(rClosed);
        }


        if (theOpMode.gamepad1.y || theOpMode.gamepad2.y) {
            collection.setPower(.7);
        } else if (theOpMode.gamepad1.x || theOpMode.gamepad2.x) {
            collection.setPower(-.6);
        } else {
            collection.setPower(0);
        }


        if (theOpMode.gamepad1.dpad_left || theOpMode.gamepad2.dpad_left) {
            box.setPosition(.1);
        } else if (theOpMode.gamepad1.dpad_right || theOpMode.gamepad2.dpad_right) {
            box.setPosition(.85);
        }
        if (theOpMode.gamepad2.a) {
            hang.setPower(.8);
        } else if (theOpMode.gamepad2.b) {
            hang.setPower(-.8);
        } else {
            hang.setPower(0);
        }

        if (theOpMode.gamepad2.left_bumper) {
            hangS.setPower(.8);
        } else if (theOpMode.gamepad2.right_bumper) {
            hangS.setPower(-.8);
        } else {
            hangS.setPower(0);
        }

    }

    public void colState(collectionState state) {
        switch (state) {
            case OPEN:
                rCollection.setPosition(lOpen);
                lCollection.setPosition(rOpen);
                break;
            case CLOSED:
                rCollection.setPosition(lClosed);
                lCollection.setPosition(rClosed);
                break;
            case WIDE:
                rCollection.setPosition(.78);
                lCollection.setPosition(.69);
        }

    }

    public void arm(double power, int target, double timeoutS) {
        runtime.reset();
        collection.setTargetPosition(collection.getCurrentPosition() + target);
        collection.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        collection.setPower(Math.abs(power));
        while (((LinearOpMode) theOpMode).opModeIsActive() && runtime.seconds() < timeoutS && collection.isBusy()) {
            theOpMode.telemetry.addData("target", target);
            theOpMode.telemetry.addData("current Position", collection.getCurrentPosition());
            theOpMode.telemetry.update();
        }
        collection.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        collection.setPower(0);
    }

    public void auto() {
        collectionDone = false;
        while (((LinearOpMode) theOpMode).opModeIsActive() && !collectionDone) {
            switch (autoState) {
                case START:
                    if (collectionStart) {
                        autoState = AutoState.DELIVER;
                        transferTimer.reset();

                    }
                    break;
                case COLLECT:
                    rCollection.setPosition(lClosed);
                    lCollection.setPosition(lOpen);

                    if (transferTimer.seconds() >= 2) {
                        autoState = AutoState.DELIVER;
                    }
                    break;
                case DELIVER:
                    target = 10;
                    if (Math.abs(collection.getCurrentPosition() - target) < 10) {
                        collectionTimer.reset();
                        rCollection.setPosition(lOpen);
                        lCollection.setPosition(rOpen);
//                        autoState = AutoState.RETURN;
//                    }
//                case RETURN:
//                    //target = 100;
//                        if (Math.abs(collection.getCurrentPosition() - target) < 15) {
                            autoState = AutoState.START;
                            collectionDone = true;
                            collectionStart = false;
                        }

                    break;

                default:
                    autoState = AutoState.START;
            }


            controller.setPID(p, i, d);
            int curPos = collection.getCurrentPosition();
            double pid = controller.calculate(curPos, target);
            double ff = Math.cos(Math.toRadians(target / ticksPerInch)) * f;
            double power = pid + ff;
            collection.setPower(power);
            theOpMode.telemetry.addData("pos", curPos);
            theOpMode.telemetry.addData("target", target);
            theOpMode.telemetry.addData("power", power);
            theOpMode.telemetry.addData("collection state", autoState);
            theOpMode.telemetry.addData("collection timer", collectionTimer);
            theOpMode.telemetry.update();
        }

    }
    public void pidTest(double timeoutS) {
        target = 150;
        controller.setPID(p, i, d);
        int curPos = collection.getCurrentPosition();
        double pid = controller.calculate(curPos, target);
        double ff = Math.cos(Math.toRadians(target / ticksPerInch)) * f;
        double power = pid + ff;
        collection.setPower(power);
        while (((LinearOpMode) theOpMode).opModeIsActive() && runtime.seconds() < timeoutS && collection.isBusy()) {
            theOpMode.telemetry.addData("pos", curPos);
            theOpMode.telemetry.addData("target", target);
            theOpMode.telemetry.addData("power", power);
            theOpMode.telemetry.addData("collection state", autoState);
            theOpMode.telemetry.update();

        }
    }
    public void motorDown() {
        collection.setPower(-.5);
        while (((LinearOpMode) theOpMode).opModeIsActive() && !collection.isOverCurrent()) {
            theOpMode.telemetry.addData("current current", collection.getCurrent(CurrentUnit.AMPS));
            theOpMode.telemetry.update();
        }
        collection.setPower(0);
    }
    public void motorUp() {
        collection.setPower(.6);
        while (((LinearOpMode) theOpMode).opModeIsActive() && !collection.isOverCurrent()) {
            theOpMode.telemetry.addData("current current", collection.getCurrent(CurrentUnit.AMPS));
            theOpMode.telemetry.update();
        }
        collection.setPower(0);
    }
}
