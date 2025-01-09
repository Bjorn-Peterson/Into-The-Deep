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
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
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
        RESET

    }

    ExtendState extendState = ExtendState.START;

    private PIDController controller;
    public static double p = 0.011, i = 0.00001, d = 0.00008;
    public static int target;


    private OpMode theOpMode;
    DcMotorEx extend;
    DcMotorEx collection;
    public Servo rCollection;
    public Servo lCollection;
    public Servo claw;
    public Servo deliveryS;

    int extended = 655;
    int retracted = 0;
    int mid = 400;
    int shortPos = 100;
    double collect = .653;
    double transfer = .55;
    double xHeight = .5;
    double initPos = .35;

    double closed = .87;
    double open = .754;
    double specClosed = .83;
    double frontSpec = .17;
    double backSpec = .62;
    double transferPos = .2;
    double midPos = .45;
    double lowerMid = .35;
    DigitalChannel cBeam;
    DigitalChannel dBeam;
    DigitalChannel lSwitch;
    DigitalChannel touch;
    NormalizedColorSensor colorSensor;
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
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        colorSensor.getNormalizedColors();
        touch = hardwareMap.get(DigitalChannel.class, "touch");
        touch.setMode(DigitalChannel.Mode.INPUT);

        cBeam = hardwareMap.get(DigitalChannel.class, "beam");
        cBeam.setMode(DigitalChannel.Mode.INPUT);
        dBeam = hardwareMap.get(DigitalChannel.class, "dBeam");
        dBeam.setMode(DigitalChannel.Mode.INPUT);
        lSwitch = hardwareMap.get(DigitalChannel.class, "lSwitch");
        lSwitch.setMode(DigitalChannel.Mode.INPUT);
    }

    public void tele() {
        if (theOpMode.gamepad1.left_bumper) {
            claw.setPosition(open);
        } else if (theOpMode.gamepad1.right_bumper) {
            claw.setPosition(closed);
        }
        if (theOpMode.gamepad1.dpad_up) {
            // deliveryState = DeliveryState.DELIVER;
            deliveryS.setPosition(backSpec);
        } else if (theOpMode.gamepad1.dpad_down) {
            // deliveryState = DeliveryState.COLLECT;
            deliveryS.setPosition(transferPos);
        }

        if (theOpMode.gamepad1.b) {
            extendState = ExtendState.EJECT;
        }
        if (theOpMode.gamepad1.ps) {
            extendState = ExtendState.RESET;
        }
            sensorReading = colorSensor.getNormalizedColors().red;
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
                    target = -700;
                    if (!touch.getState()) {
                        extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        target = 10;
                        extendState = ExtendState.START;
                    }
                    break;
                // Rotate to collecting position
                case SPECIMEN:
                    deliveryS.setPosition(backSpec);
                    claw.setPosition(open);
                    if (!dBeam.getState()) {
                        claw.setPosition(specClosed);
                        deliveryS.setPosition(frontSpec);
                        extendState = ExtendState.START;
                    }
                    break;

                case EXTEND:
                    if (Math.abs(extend.getCurrentPosition() - extended) < 100 && theOpMode.gamepad1.x) {
                        extendState = ExtendState.EXTENDED;
                        collection.setPower(.95);
                        rCollection.setPosition(collect);
                        lCollection.setPosition(collect);
                    }
                    break;
                case EXTENDED:
                case MID:

                    // If we collect a specimen, retract extension, rotate to transfer position, stop collection

                    if (!cBeam.getState()) {
                        deliveryS.setPosition(transferPos);
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
                    if (sensorReading <= blueThreshold) {
                        extendState = ExtendState.EXTEND;
                    }
                    deliveryS.setPosition(transferPos);
                    claw.setPosition(open);
                    if (Math.abs(extend.getCurrentPosition() - retracted) < 40) {
                        rCollection.setPosition(transfer);
                        lCollection.setPosition(transfer);
                        if (Math.abs(extend.getCurrentPosition() - retracted) < 20 && !lSwitch.getState()) {
                            collection.setPower(.65);
                        }
                    }
                    if (!dBeam.getState()) {
                        collection.setPower(-.9);
                        transferTimer.reset();
                        claw.setPosition(closed);
                        extendState = ExtendState.TRANSFER;
                        theOpMode.telemetry.addData("DBeam", "Broken");
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
            theOpMode.telemetry.addData("pos", curPos);
            theOpMode.telemetry.addData("target", target);
            theOpMode.telemetry.addData("Current State", extendState);
            theOpMode.telemetry.addData("power", power);
//        theOpMode.telemetry.addData("delivery position", deliveryS.getPosition());
            if (!dBeam.getState()) {
                theOpMode.telemetry.addData("Beam", "Broken");
            } else {
                theOpMode.telemetry.addData("Beam", "Not Broken");
            }
            theOpMode.telemetry.update();


        }


        public void blueTele() {
                if (theOpMode.gamepad1.left_bumper) {
                    claw.setPosition(open);
                } else if (theOpMode.gamepad1.right_bumper) {
                    claw.setPosition(closed);
                }
                if (theOpMode.gamepad1.dpad_up) {
                    // deliveryState = DeliveryState.DELIVER;
                    deliveryS.setPosition(backSpec);
                } else if (theOpMode.gamepad1.dpad_down) {
                    // deliveryState = DeliveryState.COLLECT;
                    deliveryS.setPosition(transferPos);
                }

                if (theOpMode.gamepad1.b) {
                    extendState = ExtendState.EJECT;
                }
                if (theOpMode.gamepad1.ps) {
                    extendState = ExtendState.RESET;
                }
                sensorReading = colorSensor.getNormalizedColors().blue;
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
                        target = -700;
                        if (!touch.getState()) {
                            extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                            target = 10;
                            extendState = ExtendState.START;
                        }
                        break;
                    // Rotate to collecting position
                    case SPECIMEN:
                        deliveryS.setPosition(backSpec);
                        claw.setPosition(open);
                        if (!dBeam.getState()) {
                            claw.setPosition(specClosed);
                            deliveryS.setPosition(frontSpec);
                            extendState = ExtendState.START;
                        }
                        break;

                    case EXTEND:
                        if (Math.abs(extend.getCurrentPosition() - extended) < 100 && theOpMode.gamepad1.x) {
                            extendState = ExtendState.EXTENDED;
                            collection.setPower(.95);
                            rCollection.setPosition(collect);
                            lCollection.setPosition(collect);
                        }
                        break;
                    case EXTENDED:
                    case MID:

                        // If we collect a specimen, retract extension, rotate to transfer position, stop collection

                        if (!cBeam.getState()) {
                            deliveryS.setPosition(transferPos);
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
                        if (sensorReading >= redThreshold) {
                            extendState = ExtendState.EXTEND;
                        }
                        deliveryS.setPosition(transferPos);
                        claw.setPosition(open);
                        if (Math.abs(extend.getCurrentPosition() - retracted) < 40) {
                            rCollection.setPosition(transfer);
                            lCollection.setPosition(transfer);
                            if (Math.abs(extend.getCurrentPosition() - retracted) < 20 && !lSwitch.getState()) {
                                collection.setPower(.65);
                            }
                        }
                        if (!dBeam.getState()) {
                            collection.setPower(-.9);
                            transferTimer.reset();
                            claw.setPosition(closed);
                            extendState = ExtendState.TRANSFER;
                            theOpMode.telemetry.addData("DBeam", "Broken");
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
                theOpMode.telemetry.addData("pos", curPos);
                theOpMode.telemetry.addData("target", target);
                theOpMode.telemetry.addData("Current State", extendState);
                theOpMode.telemetry.addData("power", power);
//        theOpMode.telemetry.addData("delivery position", deliveryS.getPosition());
                if (!dBeam.getState()) {
                    theOpMode.telemetry.addData("Beam", "Broken");
                } else {
                    theOpMode.telemetry.addData("Beam", "Not Broken");
                }
                theOpMode.telemetry.update();


            }






    public void test() {
        if (!dBeam.getState()) {
            theOpMode.telemetry.addData("Beam", "Broken");
        }
        else {
            theOpMode.telemetry.addData("Beam", "Not Broken");
        }
    }
    public void testColor(boolean isRed) {
        if (isRed) {
            sensorReading = colorSensor.getNormalizedColors().red;
            threshold = redThreshold;

        } else {
            sensorReading = colorSensor.getNormalizedColors().blue;
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

                if (!cBeam.getState() && sensorReading <= threshold) {
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
        if (theOpMode.gamepad1.a && extendState !=  ExtendState.START) {
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
       theOpMode.telemetry.addData("sensorReading", sensorReading);
        theOpMode.telemetry.update();


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
                    target = 655;
                    deliveryS.setPosition(lowerMid);
                    collection.setPower(.95);
                    rCollection.setPosition(collect);
                    lCollection.setPosition(collect);
                    if (Math.abs(extend.getCurrentPosition() - target) < 100) {

                        collection.setPower(.95);
                        beamTimer.reset();
                        failTimer.reset();
                        extendState = ExtendState.EXTENDED;
                    }
                    break;
                case EXTENDED:
                    if (beamTimer.seconds() > 0.72) {
                        target = shortPos;
                        lCollection.setPosition(transfer);
                        rCollection.setPosition(transfer);
                        if (beamTimer.seconds() > .93) {
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
                        deliveryS.setPosition(transferPos);
                        rCollection.setPosition(xHeight);
                        lCollection.setPosition(xHeight);
                        //target = retracted;
                        collection.setPower(0);
                        extendState = ExtendState.RETRACT;
                        beamTimer.reset();





                    }
                    break;
                case RETRACT:
                    deliveryS.setPosition(transferPos);
                    claw.setPosition(open);
                    if (beamTimer.seconds() >= 1.5 && cBeam.getState()) {
                        collection.setPower(-.95);
                        extendState = ExtendState.EXTEND;
                    }
                    if (Math.abs(extend.getCurrentPosition() - retracted) < 40) {
                        rCollection.setPosition(transfer);
                        lCollection.setPosition(transfer);
                        if (Math.abs(extend.getCurrentPosition() - retracted) < 20 && !lSwitch.getState()) {
                            collection.setPower(.73);

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
            }
            else {
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
                    target = 280;
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
                    target = -15;
                    if (Math.abs(extend.getCurrentPosition() - target) < 40) {
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
                    target = 650;
                    deliveryS.setPosition(lowerMid);
                    collection.setPower(.95);

                    if (Math.abs(extend.getCurrentPosition() - target) < 250) {
                        rCollection.setPosition(collect);
                        lCollection.setPosition(collect);
                        collection.setPower(.95);
                        beamTimer.reset();
                        failTimer.reset();
                        extendState = ExtendState.EXTENDED;
                    }
                    break;
                case EXTENDED:
                    if (beamTimer.seconds() > 0.7) {
                        target = shortPos;
                        lCollection.setPosition(transfer);
                        rCollection.setPosition(transfer);
                        if (beamTimer.seconds() > .93) {
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
                        deliveryS.setPosition(transferPos);
                        rCollection.setPosition(xHeight);
                        lCollection.setPosition(xHeight);
                        //target = retracted;
                        collection.setPower(0);
                        extendState = ExtendState.RETRACT;
                        beamTimer.reset();





                    }
                    break;
                case RETRACT:
                    deliveryS.setPosition(transferPos);
                    claw.setPosition(open);
                    if (beamTimer.seconds() >= 1.2 && cBeam.getState()) {
                        extendState = ExtendState.EXTEND;
                    }
                    if (Math.abs(extend.getCurrentPosition() - retracted) < 40) {
                        rCollection.setPosition(transfer);
                        lCollection.setPosition(transfer);
                        if (Math.abs(extend.getCurrentPosition() - retracted) < 20 && !lSwitch.getState()) {
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
                    //collection.setPower(-.8);
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
            }
            else {
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
                    target = 655;
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
}


