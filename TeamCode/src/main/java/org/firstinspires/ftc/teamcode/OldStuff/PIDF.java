package org.firstinspires.ftc.teamcode.OldStuff;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class PIDF {


    public enum ExtendState {
        START,
        RETRACT,
        MID,
        EXTEND,
        EXTENDED,
        EJECT,
        TRANSFER,
        SPECIMEN,
        RESET,
        REJECT,
        DELIVER,
        SPEC

    }

    ExtendState extendState = ExtendState.START;

    PIDController controller;
    public static double p = 0.011, i = 0.0001, d = 0.00008;
    public static int target;


    OpMode theOpMode;
    DcMotorEx extend;
    DcMotorEx collection;
    public Servo rCollection;
    public Servo lCollection;
    public Servo claw;
    public Servo deliveryS;

    int extended = 565;
    int retracted = 20;
    int mid = 300;
    int shortPos = 100;
    double collect = .68;
    double transfer = .54;
    double xHeight = .5;
    double initPos = .35;

    double closed = .87;
    double open = .754;
    double specClosed = .83;
    double frontSpec = .05;
    double backSpec = .54;
    double transferPos = .085;
    double midPos = .33;
    double lowerMid = .15;
    double specPos = .5;
    double midMid = .22;
    DigitalChannel cBeam;
    DigitalChannel dBeam;
    DigitalChannel lSwitch;
    DigitalChannel touch;
    ColorSensor colorSensor;
    double sensorReading;
    ElapsedTime transferTimer = new ElapsedTime();
    ElapsedTime beamTimer = new ElapsedTime();
    ElapsedTime failTimer = new ElapsedTime();
    double blueThreshold = .0011;
    double redThreshold = .001;
    double threshold;

    public PIDF(HardwareMap hardwareMap, OpMode opMode) {
        theOpMode = opMode;
        controller = new PIDController(p, i, d);
        extend = hardwareMap.get(DcMotorEx.class, "extend");
        extend.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        extend.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        extend.setDirection(DcMotorSimple.Direction.REVERSE);
        lCollection = hardwareMap.get(Servo.class, "lCollection");
        rCollection = hardwareMap.get(Servo.class, "rCollection");
        rCollection.setDirection(Servo.Direction.REVERSE);
        claw = hardwareMap.get(Servo.class, "claw");
        deliveryS = hardwareMap.get(Servo.class, "delivery");
        collection = hardwareMap.get(DcMotorEx.class, "collection");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        touch = hardwareMap.get(DigitalChannel.class, "touch");
        touch.setMode(DigitalChannel.Mode.INPUT);

        cBeam = hardwareMap.get(DigitalChannel.class, "beam");
        cBeam.setMode(DigitalChannel.Mode.INPUT);
        dBeam = hardwareMap.get(DigitalChannel.class, "dBeam");
        dBeam.setMode(DigitalChannel.Mode.INPUT);
        lSwitch = hardwareMap.get(DigitalChannel.class, "lSwitch");
        lSwitch.setMode(DigitalChannel.Mode.INPUT);
    }

    public void tele(boolean isRed) {

        if (theOpMode.gamepad1.left_bumper) {
            failTimer.reset();
            claw.setPosition(open);
           // deliveryS.setPosition(backSpec);
            extendState = ExtendState.DELIVER;

        } else if (theOpMode.gamepad1.right_bumper) {
            claw.setPosition(closed);
        }
        if (theOpMode.gamepad1.dpad_up) {
            deliveryS.setPosition(backSpec);

        } else if (theOpMode.gamepad1.dpad_down) {
            deliveryS.setPosition(transferPos);
        }

        if (theOpMode.gamepad1.b) {
            extendState = ExtendState.EJECT;
        }
        if (theOpMode.gamepad1.ps) {
            extendState = ExtendState.RESET;
        }
        if (theOpMode.gamepad1.dpad_right) {
            extendState = ExtendState.SPEC;
        }
        switch (extendState) {
            case DELIVER:
                if (failTimer.seconds() >= .3) {
                    deliveryS.setPosition(midPos);
                    extendState = ExtendState.START;
                }
                break;
            case SPEC:
                target = mid;
                if (Math.abs(extend.getCurrentPosition() - mid) < 50) {
                    collection.setPower(.95);
                    rCollection.setPosition(collect);
                    lCollection.setPosition(collect);

                    if (!cBeam.getState()) {
                        deliveryS.setPosition(transferPos);
                        target = retracted;
                        collection.setPower(0);
                        rCollection.setPosition(xHeight);
                        lCollection.setPosition(xHeight);
                        extendState = ExtendState.RETRACT;
                        theOpMode.telemetry.addData("Beam", "Broken");
                    }
                    if (theOpMode.gamepad1.x) {
                        target = extended;
                        extendState = ExtendState.EXTENDED;
                    }
                }
                break;




                //Fully Retracted in transfer position
            case START:
                collection.setPower(0);
                rCollection.setPosition(xHeight);
                lCollection.setPosition(xHeight);

                if (theOpMode.gamepad1.dpad_left) {
                    extendState = ExtendState.SPECIMEN;
                }
                // Start extending, turn on collection
                if (theOpMode.gamepad1.x) {
                    target = extended;
                    rCollection.setPosition(xHeight);
                    lCollection.setPosition(xHeight);
                    extendState = ExtendState.EXTEND;
                }
                if (theOpMode.gamepad1.y) {
                    target = mid;
                    rCollection.setPosition(collect);
                    lCollection.setPosition(collect);
                    extendState = ExtendState.MID;
                    collection.setPower(.95);
                }


                break;
            case RESET:
                target = -600;
                if (!touch.getState()) {
                    extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    target = 0;
                    extendState = ExtendState.START;
                }
                break;
            // Rotate to collecting position
            case SPECIMEN:
                deliveryS.setPosition(specPos);
                claw.setPosition(open);
                if (!dBeam.getState()) {
                    claw.setPosition(specClosed);
                    deliveryS.setPosition(frontSpec);
                    extendState = ExtendState.START;
                }
                break;

            case EXTEND:
                target = extended;
                if (Math.abs(extend.getCurrentPosition() - extended) < 180 && theOpMode.gamepad1.x) {
                    extendState = ExtendState.EXTENDED;
                    collection.setPower(.95);
                    rCollection.setPosition(collect);
                    lCollection.setPosition(collect);
                }
                if (!dBeam.getState()) {
                    collection.setPower(-.9);
                    transferTimer.reset();
                    claw.setPosition(closed);
                    extendState = ExtendState.TRANSFER;
                    theOpMode.telemetry.addData("DBeam", "Broken");
                    theOpMode.telemetry.update();
                }

                break;
            case EXTENDED:
            case MID:

                // If we collect a specimen, retract extension, rotate to transfer position, stop collection
                collection.setPower(.95);
                rCollection.setPosition(collect);
                lCollection.setPosition(collect);

                if (!cBeam.getState()) {
                    deliveryS.setPosition(midMid);
                    target = retracted;
                    collection.setPower(0);
                    rCollection.setPosition(xHeight);
                    lCollection.setPosition(xHeight);
                    extendState = ExtendState.RETRACT;
                    theOpMode.telemetry.addData("Beam", "Broken");

                }
                if (theOpMode.gamepad1.y) {
                    target = mid;
                    rCollection.setPosition(collect);
                    lCollection.setPosition(collect);
                    extendState = ExtendState.MID;
                    collection.setPower(.9);
                }
                break;
            case RETRACT:

                if (((double) colorSensor.red() / colorSensor.blue()) >= 2 && ((double) colorSensor.red() / colorSensor.alpha() >= 1.3) && !isRed && (Math.abs(extend.getCurrentPosition())  >= 100)) {
                    collection.setPower(.6);
                    extendState = ExtendState.EXTEND;
                }
                if ((double) colorSensor.blue() / colorSensor.red() >= 1.9 && isRed && (Math.abs(extend.getCurrentPosition())  >= 100)) {
                    collection.setPower(.6);
                    extendState = ExtendState.EXTEND;
                }



                claw.setPosition(open);
                if (Math.abs(extend.getCurrentPosition() - retracted) < 40) {
                    deliveryS.setPosition(transferPos);
                    rCollection.setPosition(transfer);
                    lCollection.setPosition(transfer);
                    if (Math.abs(extend.getCurrentPosition() - retracted) < 25 && !lSwitch.getState()) {
                        collection.setPower(.65);
                    }
                }
                if (!dBeam.getState()) {
                    collection.setPower(-.9);
                    transferTimer.reset();
                    claw.setPosition(closed);
                    extendState = ExtendState.TRANSFER;
                    theOpMode.telemetry.addData("DBeam", "Broken");
                    theOpMode.telemetry.update();
                }

                break;
            case TRANSFER:
                theOpMode.telemetry.addData("transferTimer", transferTimer);
                theOpMode.telemetry.update();
                target = shortPos;
                if (transferTimer.seconds() >= .2) {
                    collection.setPower(0);
                    target = retracted;
                    extendState = ExtendState.START;
                    claw.setPosition(closed);
                    deliveryS.setPosition(backSpec);
                }
                break;
            case EJECT:
                if (theOpMode.gamepad1.b) {
                    collection.setPower(-.6);
                } else {
                    collection.setPower(0);
                    extendState = ExtendState.START;
                }
                break;
            case REJECT:
                target = extended;
                collection.setPower(-.6);
                if (Math.abs(extend.getCurrentPosition() - extended) < 50 && cBeam.getState()) {
                    collection.setPower(.95);
                    rCollection.setPosition(collect);
                    lCollection.setPosition(collect);
                    extendState = ExtendState.EXTENDED;
                }
                break;

            default:
                extendState = ExtendState.START;
        }
        if (theOpMode.gamepad1.a && extendState != ExtendState.START) {
            rCollection.setPosition(transfer);
            lCollection.setPosition(transfer);
            collection.setPower(0);
            target = retracted;
            extendState = ExtendState.START;
        }
        controller.setPID(p, i, d);
        int curPos = extend.getCurrentPosition();
        double power = controller.calculate(curPos, target);
        extend.setPower(power * .9);
        theOpMode.telemetry.addData("pos", curPos);
        theOpMode.telemetry.addData("target", target);
        theOpMode.telemetry.addData("Current State", extendState);
        theOpMode.telemetry.addData("power", power);
        theOpMode.telemetry.addData("failTimer", failTimer);
//        theOpMode.telemetry.addData("delivery position", deliveryS.getPosition());
        if (!dBeam.getState()) {
            theOpMode.telemetry.addData("Beam", "Broken");
        } else {
            theOpMode.telemetry.addData("Beam", "Not Broken");
        }
        theOpMode.telemetry.update();


    }


    public void testColor(boolean isRed) {
        if (isRed) {
            sensorReading = colorSensor.red();
            threshold = redThreshold;

        } else {
            sensorReading = colorSensor.blue();
            threshold = blueThreshold;
        }

        switch (extendState) {

            //Fully Retracted in transfer position
            case START:
                collection.setPower(0);
                rCollection.setPosition(xHeight);
                lCollection.setPosition(xHeight);

                if (theOpMode.gamepad1.dpad_left) {
                    extendState = ExtendState.SPECIMEN;
                }
                // Start extending, turn on collection

                if (theOpMode.gamepad1.y) {
                    target = mid;
                    rCollection.setPosition(collect);
                    lCollection.setPosition(collect);
                    extendState = ExtendState.MID;
                    collection.setPower(.95);
                }


                break;
            case MID:

                // If we collect a specimen, retract extension, rotate to transfer position, stop collection

                if (!cBeam.getState()) {
                    deliveryS.setPosition(transferPos);
                    target = retracted;
                    collection.setPower(0);
                    rCollection.setPosition(xHeight);
                    lCollection.setPosition(xHeight);
                    extendState = ExtendState.RETRACT;

                }
                break;
            default:
                extendState = ExtendState.START;
        }
        if (theOpMode.gamepad1.a && extendState != ExtendState.START) {
            rCollection.setPosition(transfer);
            lCollection.setPosition(transfer);
            collection.setPower(0);
            target = retracted;
            extendState = ExtendState.START;
        }
        controller.setPID(p, i, d);
        int curPos = extend.getCurrentPosition();
        double power = controller.calculate(curPos, target);
        extend.setPower(power);

//        theOpMode.telemetry.addData("delivery position", deliveryS.getPosition());
        theOpMode.telemetry.addData("alpha", colorSensor.alpha());
        theOpMode.telemetry.addData("How much red?", colorSensor.red());
        theOpMode.telemetry.addData("How Much Blue?", colorSensor.blue());
        theOpMode.telemetry.addData("IsRed?", isRed);
        theOpMode.telemetry.update();


        //If We are Red
        if ((double) colorSensor.blue() / colorSensor.red() >= 1.5) {
//If too much blue is sensed, we spit it out, otherwise we keep collecting :)
            theOpMode.telemetry.addData("Blue", "");
            theOpMode.telemetry.update();
        }


        //If We are Blue
        if (((double) colorSensor.red() / colorSensor.blue()) >= 1.5 && ((double) colorSensor.red() / colorSensor.alpha() >= 1.2)) {
            //If too much red is sensed AND we aren't sensing a big spike in alpha values, we spit it out
            theOpMode.telemetry.addData("Red", "");
            theOpMode.telemetry.update();
        }


    }


    public class CollectRun implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            switch (extendState) {
                //Fully Retracted in transfer position
                case START:
                    collection.setPower(0);
                    extendState = ExtendState.EXTEND;
                    deliveryS.setPosition(midPos);
                    rCollection.setPosition(collect);
                    lCollection.setPosition(collect);

                    break;
                // Rotate to collecting position
                case EXTEND:
                    target = extended;
                    deliveryS.setPosition(lowerMid);
                    rCollection.setPosition(collect);
                    lCollection.setPosition(collect);
                    if (Math.abs(extend.getCurrentPosition() - target) < 300) {

                        collection.setPower(.95);
                        beamTimer.reset();
                        failTimer.reset();
                        extendState = ExtendState.EXTENDED;
                    }
                    break;
                case EXTENDED:
                    if (beamTimer.seconds() > 0.8) {
                        target = shortPos;
                        lCollection.setPosition(transfer);
                        rCollection.setPosition(transfer);
                        if (beamTimer.seconds() > 1.1) {
                            lCollection.setPosition(collect);
                            rCollection.setPosition(collect);
                            target = extended;
                            beamTimer.reset();
                        }
                        if (failTimer.seconds() >= 2.7) {
                            extend.setPower(0);
                            collection.setPower(0);
                            target = retracted;
                            extendState = ExtendState.START;
                            deliveryS.setPosition(midPos);
                            return false;
                        }
                    }

                    // If we collect a specimen, retract extension, rotate to transfer position, stop collection
                    if (!cBeam.getState()) {
                        target = retracted;
                        rCollection.setPosition(xHeight);
                        lCollection.setPosition(xHeight);
                        //target = retracted;
                        collection.setPower(0);
                        extendState = ExtendState.RETRACT;
                        beamTimer.reset();


                    }
                    if (!dBeam.getState()) {
                        rCollection.setPosition(xHeight);
                        lCollection.setPosition(xHeight);
                        collection.setPower(-.95);
                        transferTimer.reset();
                        claw.setPosition(closed);
                        extendState = ExtendState.TRANSFER;
                    }
                    break;
                case RETRACT:
                    claw.setPosition(open);
                    if (beamTimer.seconds() >= 1.5) {
                        collection.setPower(-.95);
                        extendState = ExtendState.EXTEND;
                    }
                    if (Math.abs(extend.getCurrentPosition() - retracted) < 40) {
                        deliveryS.setPosition(transferPos);
                        rCollection.setPosition(transfer);
                        lCollection.setPosition(transfer);
                        if (Math.abs(extend.getCurrentPosition() - retracted) < 20 && !lSwitch.getState()) {
                            collection.setPower(.66);

                        }


                        if (!dBeam.getState()) {
                            rCollection.setPosition(xHeight);
                            lCollection.setPosition(xHeight);
                            collection.setPower(-.9);
                            transferTimer.reset();
                            claw.setPosition(closed);
                            extendState = ExtendState.TRANSFER;
                        }
                    }


                    break;
                case TRANSFER:
                    target = shortPos;
                    if (transferTimer.seconds() >= .1) {
                        extend.setPower(0);
                        collection.setPower(0);
                        target = retracted;
                        extendState = ExtendState.START;
                        claw.setPosition(closed);
                        deliveryS.setPosition(midPos);
                        return false;
                    }
                    break;

                default:
                    extendState = ExtendState.START;
            }
            controller.setPID(p, i, d);
            int curPos = extend.getCurrentPosition();
            double power = controller.calculate(curPos, target);
            extend.setPower(power);
            theOpMode.telemetry.addData("Current State", extendState);
            theOpMode.telemetry.addData("Beam timer", beamTimer);
            if (!dBeam.getState()) {
                theOpMode.telemetry.addData("Beam", "Broken");
            } else {
                theOpMode.telemetry.addData("Beam", "Not Broken");
            }
            theOpMode.telemetry.update();
            return true;
        }
    }

    public Action collectRun() {
        return new CollectRun();
    }

    public class InitPositions implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            rCollection.setPosition(initPos);
            lCollection.setPosition(initPos);
            deliveryS.setPosition(midPos);
            return false;
        }
    }

    public Action initPositions() {
        return new InitPositions();
    }

    public class ExtendCollection implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            switch (extendState) {
                case START:
                    extendState = ExtendState.EXTEND;
                    rCollection.setPosition(transfer);
                    lCollection.setPosition(transfer);
                case EXTEND:
                    target = 230;
                    if (Math.abs(extend.getCurrentPosition() - target) < 20) {
                        extend.setPower(0);
                        extendState = ExtendState.START;
                        return false;
                    }
            }
            controller.setPID(p, i, d);
            int curPos = extend.getCurrentPosition();
            double power = controller.calculate(curPos, target);
            extend.setPower(power);
            theOpMode.telemetry.addData("pos", curPos);
            theOpMode.telemetry.addData("target", target);
            theOpMode.telemetry.addData("Current State", extendState);
            theOpMode.telemetry.addData("power", power);
            theOpMode.telemetry.update();
            return true;
        }
    }

    public Action extendCollection() {
        return new ExtendCollection();
    }

    public class RetractCollection implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            switch (extendState) {
                case START:
                    extendState = ExtendState.EXTEND;
                    rCollection.setPosition(xHeight);
                    lCollection.setPosition(xHeight);
                case EXTEND:
                    target = 0;
                    if (Math.abs(extend.getCurrentPosition() - target) < 40) {
                        extend.setPower(0);
                        extendState = ExtendState.START;
                        theOpMode.telemetry.addData("Finished", "");
                        theOpMode.telemetry.update();
                        return false;
                    }
            }
            controller.setPID(p, i, d);
            int curPos = extend.getCurrentPosition();
            double power = controller.calculate(curPos, target);
            extend.setPower(power);
            return true;
        }
    }

    public Action retractCollection() {
        return new RetractCollection();
    }

    public class SubCollect implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            switch (extendState) {
                //Fully Retracted in transfer position
                case START:
                    collection.setPower(0);
                    extendState = ExtendState.EXTEND;
                    deliveryS.setPosition(midPos);


                    break;
                // Rotate to collecting position
                case EXTEND:
                    target = extended;
                    deliveryS.setPosition(lowerMid);
                    collection.setPower(.95);

                    if (Math.abs(extend.getCurrentPosition() - target) < 350) {
                        rCollection.setPosition(collect);
                        lCollection.setPosition(collect);
                        collection.setPower(.95);
                        beamTimer.reset();
                        failTimer.reset();
                        extendState = ExtendState.EXTENDED;
                    }
                    if (!dBeam.getState()) {
                        rCollection.setPosition(xHeight);
                        lCollection.setPosition(xHeight);
                        collection.setPower(-.95);
                        transferTimer.reset();
                        claw.setPosition(closed);
                        extendState = ExtendState.TRANSFER;
                    }
                    break;
                case EXTENDED:
                    target = extended;
                    if (beamTimer.seconds() > 0.8) {
                        target = mid;
                        lCollection.setPosition(transfer);
                        rCollection.setPosition(transfer);
                        if (beamTimer.seconds() > 1.1) {
                            lCollection.setPosition(collect);
                            rCollection.setPosition(collect);
                            target = extended;
                            beamTimer.reset();
                        }
                        if (failTimer.seconds() >= 3.4) {
                            extend.setPower(0);
                            collection.setPower(0);
                            target = retracted;
                            extendState = ExtendState.START;
                            deliveryS.setPosition(midPos);
                            return false;
                        }
                    }

                    // If we collect a sample, retract extension, rotate to transfer position, stop collection
                    if (!cBeam.getState()) {
                        target = retracted;
                        rCollection.setPosition(xHeight);
                        lCollection.setPosition(xHeight);
                        //target = retracted;
                        collection.setPower(0);
                        extendState = ExtendState.RETRACT;
                        beamTimer.reset();


                    }
                    if (!dBeam.getState()) {
                        rCollection.setPosition(xHeight);
                        lCollection.setPosition(xHeight);
                        collection.setPower(-.95);
                        transferTimer.reset();
                        claw.setPosition(closed);
                        extendState = ExtendState.TRANSFER;
                    }
                    break;
                case RETRACT:
                    if ((double) colorSensor.blue() / colorSensor.red() >= 1.8 && (Math.abs(extend.getCurrentPosition())  >= 80)) {
                        extendState = ExtendState.EXTEND;
                    }

                    claw.setPosition(open);
                    if (beamTimer.seconds() >= 1.2 && cBeam.getState()) {
                        extendState = ExtendState.EXTEND;
                    }
                    if (Math.abs(extend.getCurrentPosition() - retracted) < 40) {
                        deliveryS.setPosition(transferPos);
                        rCollection.setPosition(transfer);
                        lCollection.setPosition(transfer);
                        if (Math.abs(extend.getCurrentPosition() - retracted) < 25 && !lSwitch.getState()) {
                            collection.setPower(.73);

                        }


                        if (!dBeam.getState()) {
                            rCollection.setPosition(xHeight);
                            lCollection.setPosition(xHeight);
                            collection.setPower(-.95);
                            transferTimer.reset();
                            claw.setPosition(closed);
                            extendState = ExtendState.TRANSFER;
                        }
                    }


                    break;
                case TRANSFER:
                    target = shortPos;
                    if (transferTimer.seconds() >= .17) {
                        extend.setPower(0);
                        collection.setPower(0);
                        target = retracted;
                        extendState = ExtendState.START;
                        claw.setPosition(closed);
                        deliveryS.setPosition(midPos);
                        return false;
                    }
                    break;
                case REJECT:
                    target = 450;
                    collection.setPower(-.6);
                    if (Math.abs(extend.getCurrentPosition() - extended) < 200 && cBeam.getState()) {
                        collection.setPower(.95);
                        rCollection.setPosition(collect);
                        lCollection.setPosition(collect);
                        extendState = ExtendState.EXTENDED;
                    }
                    break;

                default:
                    extendState = ExtendState.START;
            }
            controller.setPID(p, i, d);
            int curPos = extend.getCurrentPosition();
            double power = controller.calculate(curPos, target);
            extend.setPower(power);
            theOpMode.telemetry.addData("Current State", extendState);
            theOpMode.telemetry.addData("Beam timer", beamTimer);
            if (!dBeam.getState()) {
                theOpMode.telemetry.addData("Beam", "Broken");
            } else {
                theOpMode.telemetry.addData("Beam", "Not Broken");
            }
            theOpMode.telemetry.update();
            return true;
        }

    }

    public Action subCollect() {
        return new SubCollect();
    }

    public class MegaExtend implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            switch (extendState) {
                case START:
                    extendState = ExtendState.EXTEND;
                    rCollection.setPosition(transfer);
                    lCollection.setPosition(transfer);
                case EXTEND:
                    target = extended;
                    if (Math.abs(extend.getCurrentPosition() - target) < 20) {
                        extend.setPower(0);
                        extendState = ExtendState.START;
                        return false;
                    }
            }
            controller.setPID(p, i, d);
            int curPos = extend.getCurrentPosition();
            double power = controller.calculate(curPos, target);

            extend.setPower(power);
            return true;
        }
    }

    public Action megaExtend() {
        return new MegaExtend();
    }




















































































        public class SubCollectBlue implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                switch (extendState) {
                    //Fully Retracted in transfer position
                    case START:
                        collection.setPower(0);
                        extendState = ExtendState.EXTEND;
                        deliveryS.setPosition(midPos);


                        break;
                    // Rotate to collecting position
                    case EXTEND:
                        target = extended;
                        deliveryS.setPosition(lowerMid);
                        collection.setPower(.95);

                        if (Math.abs(extend.getCurrentPosition() - target) < 350) {
                            rCollection.setPosition(collect);
                            lCollection.setPosition(collect);
                            collection.setPower(.95);
                            beamTimer.reset();
                            failTimer.reset();
                            extendState = ExtendState.EXTENDED;
                        }
                        if (!dBeam.getState()) {
                            rCollection.setPosition(xHeight);
                            lCollection.setPosition(xHeight);
                            collection.setPower(-.95);
                            transferTimer.reset();
                            claw.setPosition(closed);
                            extendState = ExtendState.TRANSFER;
                        }
                        break;
                    case EXTENDED:
                        target = extended;
                        if (beamTimer.seconds() > 0.8) {
                            target = mid;
                            lCollection.setPosition(transfer);
                            rCollection.setPosition(transfer);
                            if (beamTimer.seconds() > 1.1) {
                                lCollection.setPosition(collect);
                                rCollection.setPosition(collect);
                                target = extended;
                                beamTimer.reset();
                            }
                            if (failTimer.seconds() >= 3.4) {
                                extend.setPower(0);
                                collection.setPower(0);
                                target = retracted;
                                extendState = ExtendState.START;
                                deliveryS.setPosition(midPos);
                                return false;
                            }
                        }

                        // If we collect a sample, retract extension, rotate to transfer position, stop collection
                        if (!cBeam.getState()) {
                            target = retracted;
                            rCollection.setPosition(xHeight);
                            lCollection.setPosition(xHeight);
                            //target = retracted;
                            collection.setPower(0);
                            extendState = ExtendState.RETRACT;
                            beamTimer.reset();


                        }
                        if (!dBeam.getState()) {
                            rCollection.setPosition(xHeight);
                            lCollection.setPosition(xHeight);
                            collection.setPower(-.95);
                            transferTimer.reset();
                            claw.setPosition(closed);
                            extendState = ExtendState.TRANSFER;
                        }
                        break;
                    case RETRACT:
                        if (((double) colorSensor.red() / colorSensor.blue()) >= 2 && ((double) colorSensor.red() / colorSensor.alpha() >= 1.3) && (Math.abs(extend.getCurrentPosition())  >= 80)) {
                            extendState = ExtendState.EXTEND;
                        }

                        claw.setPosition(open);
                        if (beamTimer.seconds() >= 1.2 && cBeam.getState()) {
                            extendState = ExtendState.EXTEND;
                        }
                        if (Math.abs(extend.getCurrentPosition() - retracted) < 40) {
                            deliveryS.setPosition(transferPos);
                            rCollection.setPosition(transfer);
                            lCollection.setPosition(transfer);
                            if (Math.abs(extend.getCurrentPosition() - retracted) < 25 && !lSwitch.getState()) {
                                collection.setPower(.73);

                            }


                            if (!dBeam.getState()) {
                                rCollection.setPosition(xHeight);
                                lCollection.setPosition(xHeight);
                                collection.setPower(-.95);
                                transferTimer.reset();
                                claw.setPosition(closed);
                                extendState = ExtendState.TRANSFER;
                            }
                        }


                        break;
                    case TRANSFER:
                        target = shortPos;
                        if (transferTimer.seconds() >= .17) {
                            extend.setPower(0);
                            collection.setPower(0);
                            target = retracted;
                            extendState = ExtendState.START;
                            claw.setPosition(closed);
                            deliveryS.setPosition(midPos);
                            return false;
                        }
                        break;
                    case REJECT:
                        target = 450;
                        collection.setPower(-.6);
                        if (Math.abs(extend.getCurrentPosition() - extended) < 200 && cBeam.getState()) {
                            collection.setPower(.95);
                            rCollection.setPosition(collect);
                            lCollection.setPosition(collect);
                            extendState = ExtendState.EXTENDED;
                        }
                        break;

                    default:
                        extendState = ExtendState.START;
                }
                controller.setPID(p, i, d);
                int curPos = extend.getCurrentPosition();
                double power = controller.calculate(curPos, target);
                extend.setPower(power);
                theOpMode.telemetry.addData("Current State", extendState);
                theOpMode.telemetry.addData("Beam timer", beamTimer);
                if (!dBeam.getState()) {
                    theOpMode.telemetry.addData("Beam", "Broken");
                } else {
                    theOpMode.telemetry.addData("Beam", "Not Broken");
                }
                theOpMode.telemetry.update();
                return true;
            }

        }

        public Action subCollectBlue() {
            return new SubCollectBlue();
        }
}


