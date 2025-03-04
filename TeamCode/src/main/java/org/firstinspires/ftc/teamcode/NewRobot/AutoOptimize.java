package org.firstinspires.ftc.teamcode.NewRobot;
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

import org.firstinspires.ftc.teamcode.OldStuff.PIDF;


public class AutoOptimize {


    public enum ExtendState {
        START,
        RETRACT,
        EXTEND,
        EXTENDED,
        TRANSFER,
        LIFT,
        LIFTED,
        REJECT

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
    public Servo door;

    int extended = 565;
    int retracted = 20;
    int mid = 300;
    int shortPos = 100;
    double collect = .675;
    double transfer = .54;
    double xHeight = .5;

    double closed = .58;
    double open = .39;
    double backSpec = .71;
    double transferPos = .02;
    double midPos = .54;
    double lowerMid = .15;

    DigitalChannel cBeam;
    DigitalChannel dBeam;
    DigitalChannel lSwitch;
    DigitalChannel touch;
    ColorSensor colorSensor;
    ElapsedTime transferTimer = new ElapsedTime();
    ElapsedTime beamTimer = new ElapsedTime();
    ElapsedTime failTimer = new ElapsedTime();



    PIDController liftController;
    public static double lp = 0.025, li = 0, ld = 0.001;
    public static double f = 0.021;
    public static int liftTarget = 0;
    private final double ticksPerInch = (145.1) / (1.15 * 3.14);



    DcMotorEx lift;
    DcMotorEx lift2;

    public DigitalChannel liftTouch;
    ElapsedTime liftTimer = new ElapsedTime();

    public AutoOptimize(HardwareMap hardwareMap, OpMode opMode) {
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
        door = hardwareMap.get(Servo.class, "door");
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


        liftController = new PIDController(lp, li, ld);
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        lift2 = hardwareMap.get(DcMotorEx.class, "lift2");

        lift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lift2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        liftTouch = hardwareMap.get(DigitalChannel.class, "lTouch");
        liftTouch.setMode(DigitalChannel.Mode.INPUT);
    }



    public class SpeedCollect implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            switch (extendState) {
                //Fully Retracted in transfer position
                case START:
                    door.setPosition(.15);
                    collection.setPower(0);
                    extendState = ExtendState.EXTEND;
                    deliveryS.setPosition(midPos);


                    break;
                // Rotate to collecting position
                case EXTEND:
                    target = extended;

                    if (Math.abs(extend.getCurrentPosition() - target) < 350) {
                        rCollection.setPosition(collect);
                        lCollection.setPosition(collect);
                        collection.setPower(.95);
                        beamTimer.reset();
                        failTimer.reset();
                        extendState = ExtendState.EXTENDED;
                    }

                    break;
                case EXTENDED:
                    door.setPosition(.15);
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
                        if (failTimer.seconds() >= 2.8) {
                            extend.setPower(0);
                            collection.setPower(0);
                            target = retracted;
                            extendState = ExtendState.START;
                            deliveryS.setPosition(midPos);
                            return false;
                        }
                    }
                    if (!cBeam.getState()) {
                        deliveryS.setPosition(lowerMid);
                        target = retracted;
                        rCollection.setPosition(xHeight);
                        lCollection.setPosition(xHeight);
                        //target = retracted;
                        collection.setPower(0);
                        extendState = ExtendState.RETRACT;
                        beamTimer.reset();


                    }
                    break;
                case RETRACT:
                    if ((double) colorSensor.blue() / colorSensor.red() >= 1.4 && (Math.abs(extend.getCurrentPosition())  >= 70)) {
                        beamTimer.reset();
                        collection.setPower(-.5);
                        extendState = ExtendState.REJECT;
                    }

                    claw.setPosition(open);
                    if (cBeam.getState()) {
                        extendState = ExtendState.EXTEND;
                    }
                    if (Math.abs(extend.getCurrentPosition() - retracted) < 300) {
                        collection.setPower(0);
                        target = retracted;
                        extendState = ExtendState.START;
                        return false;
                    }
                    break;
                case REJECT:
                    if (beamTimer.seconds() >= .4 && cBeam.getState()) {
                        collection.setPower(.6);
                        extendState = ExtendState.EXTEND;
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
            theOpMode.telemetry.update();
            return true;
        }

    }

    public Action speedCollect() {
        return new SpeedCollect();
    }

    public class SpeedCollect2 implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
                switch (extendState) {
                    //Fully Retracted in transfer position
                    case START:
                        collection.setPower(0);
                        extendState = ExtendState.RETRACT;

                        break;
                    // Rotate to collecting position
                    case RETRACT:
                        target = retracted;
                        claw.setPosition(open);

                        if (Math.abs(extend.getCurrentPosition() - retracted) < 55) {
                            deliveryS.setPosition(transferPos);
                            rCollection.setPosition(transfer);
                            lCollection.setPosition(transfer);
                            if (Math.abs(extend.getCurrentPosition() - retracted) < 21) {
                                collection.setPower(.5);
                                door.setPosition(.5);

                            }


                            if (!dBeam.getState()) {
                                door.setPosition(.15);
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
                        if (transferTimer.seconds() >= .1) {
                            deliveryS.setPosition(midPos);
                            extend.setPower(0);
                            collection.setPower(0);
                            target = retracted;
                            claw.setPosition(closed);
                            extendState = ExtendState.LIFT;
                        }
                        break;
                    case LIFT:
                        liftTarget = 1000;
                        if (Math.abs(lift.getCurrentPosition() - liftTarget) < 38) {
                            deliveryS.setPosition(backSpec);
                            liftTimer.reset();
                            extendState = ExtendState.LIFTED;
                        }
                        break;
                    case LIFTED:
                        if (liftTimer.seconds() >= .05) {
                            claw.setPosition(open);
                        }
                        if (liftTimer.seconds() >= .09) {
                            deliveryS.setPosition(midPos);
                        }
                        if (liftTimer.seconds() >= .12) {
                            extendState = ExtendState.START;
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


            liftController.setPID(lp, li, ld);
            int lCurPos = lift.getCurrentPosition();
            double pid = liftController.calculate(lCurPos, liftTarget);
            double ff = Math.cos(Math.toRadians(liftTarget / ticksPerInch)) * f;
            double lPower = pid + ff;
            lift.setPower(lPower);
            lift2.setPower(lPower);
            theOpMode.telemetry.addData("target", liftTarget);
            theOpMode.telemetry.addData("Current Position", lift.getCurrentPosition());
            theOpMode.telemetry.update();
            return true;

        }

    }

    public Action speedCollect2() {
        return new SpeedCollect2();
    }

}
