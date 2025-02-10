package org.firstinspires.ftc.teamcode.NewRobot;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Lift {
    public enum LiftState {
        START,
        LIFT,
        LIFTED,
        DOWN
    }

    public enum TeleState {
        START,
        LOW,
        SPECIMEN,
        MANUAL,
        UP
    }

    TeleState teleState = TeleState.START;
    LiftState liftState = LiftState.START;
    PIDController controller;
    public static double p = 0.025, i = 0, d = 0.001;
    public static double f = 0.021;
    public static int target;
    private final double ticksPerInch = (145.1) / (1.15 * 3.14);
    double closed = .87;
    double open = .754;
    double frontSpec = .32;
    double backSpec = .69;
    double midPos = .59;
    double specClosed = .83;
    double specPos = .71;


     DcMotorEx lift;
     DcMotorEx lift2;
    public Servo claw;
    public Servo deliveryS;
    public DigitalChannel liftTouch;
    ElapsedTime liftTimer = new ElapsedTime();
    ElapsedTime deliveryTimer = new ElapsedTime();
    OpMode theOpMode;
    double countsPerInch;
    DigitalChannel dBeam;



    public Lift(HardwareMap hardwareMap, OpMode opMode, double encoderTicksPerRev, double gearRatio, double wheelDiameter) {
        controller = new PIDController(p, i, d);
        countsPerInch = (encoderTicksPerRev * gearRatio) / (wheelDiameter * 3.14);
        theOpMode = opMode;
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        lift2 = hardwareMap.get(DcMotorEx.class, "lift2");

        lift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lift2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        claw = hardwareMap.get(Servo.class, "claw");
        deliveryS = hardwareMap.get(Servo.class, "delivery");
        dBeam = hardwareMap.get(DigitalChannel.class, "dBeam");
        dBeam.setMode(DigitalChannel.Mode.INPUT);
        liftTouch = hardwareMap.get(DigitalChannel.class, "lTouch");
        liftTouch.setMode(DigitalChannel.Mode.INPUT);
    }

    public void teleLift() {

        switch (teleState) {
            case START:
                if (theOpMode.gamepad1.dpad_right) {
                    teleState = TeleState.UP;
                }
                if (theOpMode.gamepad1.right_trigger > 0.1 || theOpMode.gamepad1.left_trigger > 0.1) {
                    teleState = TeleState.MANUAL;
                }
                if (!liftTouch.getState()) {
                    teleState = TeleState.LOW;
                }
                break;
            case MANUAL:

                if (theOpMode.gamepad1.right_trigger > .1) {
                    lift.setPower(theOpMode.gamepad1.right_trigger);
                    lift2.setPower(theOpMode.gamepad1.right_trigger);

                }
                else if (theOpMode.gamepad1.left_trigger > .1) {
                    lift.setPower(-theOpMode.gamepad1.left_trigger);
                    lift2.setPower(-theOpMode.gamepad1.left_trigger);

                } else {
                    lift.setPower(0);
                    lift2.setPower(0);
                }
                if (!liftTouch.getState()) {
                    teleState = TeleState.LOW;
                }
                break;
            case LOW:
                lift.setPower(0);
                lift2.setPower(0);
                if (theOpMode.gamepad1.right_trigger > .1) {
                    lift.setPower(theOpMode.gamepad1.right_trigger);
                    lift2.setPower(theOpMode.gamepad1.right_trigger);
                    teleState = TeleState.MANUAL;
                }
                break;
            case UP:
                target = 500;
                controller.setPID(p, i, d);
                int curPos = lift.getCurrentPosition();
                double pid = controller.calculate(curPos, target);
                double ff = Math.cos(Math.toRadians(target / ticksPerInch)) * f;
                double power = pid + ff;
                lift.setPower(power);
                lift2.setPower(power);
                if (lift.getCurrentPosition()-target <= 30) {
                    lift.setPower(0);
                    lift2.setPower(0);
                    target = lift.getCurrentPosition();
                    teleState = TeleState.START;
                }
            break;
            case SPECIMEN:


            default: teleState = TeleState.START;
        }

        theOpMode.telemetry.addData("LiftState", liftState);
        theOpMode.telemetry.addData("target", target);
        theOpMode.telemetry.addData("Current Position", lift.getCurrentPosition());
        theOpMode.telemetry.update();


    }


    public void soloControls() {
        switch (teleState) {
            case START:
                if (theOpMode.gamepad1.right_trigger > 0.1 || theOpMode.gamepad1.left_trigger > 0.1) {
                    teleState = TeleState.MANUAL;
                }
                if (!liftTouch.getState()) {
                    teleState = TeleState.LOW;
                }
                break;
            case MANUAL:

            if (theOpMode.gamepad1.right_trigger > .1) {
                lift.setPower(theOpMode.gamepad1.right_trigger);
                lift2.setPower(theOpMode.gamepad1.right_trigger);

            }
            else if (theOpMode.gamepad1.left_trigger > .1) {
                lift.setPower(-theOpMode.gamepad1.left_trigger);
                lift2.setPower(-theOpMode.gamepad1.left_trigger);

            } else {
                lift.setPower(0);
                lift2.setPower(0);
            }
                if (!liftTouch.getState()) {
                    teleState = TeleState.LOW;
                }
                break;
            case LOW:
                lift.setPower(0);
                lift2.setPower(0);
                if (theOpMode.gamepad1.right_trigger > .1) {
                    lift.setPower(theOpMode.gamepad1.right_trigger);
                    lift2.setPower(theOpMode.gamepad1.right_trigger);
                    teleState = TeleState.MANUAL;
                }
                break;
            default: teleState = TeleState.START;
        }

    }

    public class SpecDeliver implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            switch (liftState) {
                case START:
                    liftState = LiftState.LIFT;
                    break;
                case LIFT:
                    claw.setPosition(closed);
                    deliveryS.setPosition(backSpec);
                    target = 325;
                    if (Math.abs(lift.getCurrentPosition() - target) < 35) {
                        claw.setPosition(open);
                        lift.setPower(0);
                        lift2.setPower(0);
                        liftState = LiftState.START;
                        return false;
                    }
                    break;
                case LIFTED:
                    if (deliveryTimer.seconds() >= .1) {
                        claw.setPosition(open);
                        lift.setPower(0);
                        lift2.setPower(0);
                        liftState = LiftState.START;
                        return false;
                    }
                    break;
                default:
                    liftState = LiftState.START;

            }
            controller.setPID(p, i, d);
            int curPos = lift.getCurrentPosition();
            double pid = controller.calculate(curPos, target);
            double ff = Math.cos(Math.toRadians(target / ticksPerInch));
            double power = pid + ff;
            lift.setPower(power);
            lift2.setPower(power);
            theOpMode.telemetry.addData("LiftState", liftState);
            theOpMode.telemetry.addData("target", target);
            theOpMode.telemetry.addData("Current Position", lift.getCurrentPosition());
            theOpMode.telemetry.addData("power", power);
            theOpMode.telemetry.update();
            return true;
        }
    }

    public Action specDeliver() {
        return new SpecDeliver();
    }

    public class LiftUp implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            switch (liftState) {
                case START:
                    claw.setPosition(closed);
                    deliveryS.setPosition(midPos);
                    liftState = LiftState.LIFT;
                    break;
                case LIFT:
                    target = 980;
                    if (Math.abs(lift.getCurrentPosition() - target) < 40) {
                        deliveryS.setPosition(backSpec);
                        liftTimer.reset();
                        liftState = LiftState.LIFTED;
                    }
                    break;
                case LIFTED:
                    if (liftTimer.seconds() >= .08) {
                        claw.setPosition(open);
                    }
                    if (liftTimer.seconds() >= .14) {
                        deliveryS.setPosition(midPos);
                    }
                        if (liftTimer.seconds() >= .18) {
                            liftState = LiftState.START;
                            return false;

                    }
                        break;
                default:
                    liftState = LiftState.START;

            }
            controller.setPID(p, i, d);
            int curPos = lift.getCurrentPosition();
            double pid = controller.calculate(curPos, target);
            double ff = Math.cos(Math.toRadians(target / ticksPerInch)) * f;
            double power = pid + ff;
            lift.setPower(power);
            lift2.setPower(power);
            theOpMode.telemetry.addData("LiftState", liftState);
            theOpMode.telemetry.addData("target", target);
            theOpMode.telemetry.addData("Current Position", lift.getCurrentPosition());
            theOpMode.telemetry.update();

            return true;
        }
    }

    public Action liftUp() {
        return new LiftUp();
    }

    public class LiftDown implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            switch (liftState) {
                case START:
                    //deliveryS.setPosition(midPos);
                    liftState = LiftState.DOWN;
                    break;
                case DOWN:
                    target = -40;
                    if (Math.abs(lift.getCurrentPosition() - target) < 30 && !liftTouch.getState()) {
                        lift.setPower(-0.12);
                        lift2.setPower(-0.12);
                        liftState = LiftState.START;
                        return false;
                    }
                    break;

                    default:
                        liftState = LiftState.START;
            }
            controller.setPID(p, i, d);
            int curPos = lift.getCurrentPosition();
            double pid = controller.calculate(curPos, target);
            double ff = Math.cos(Math.toRadians(target / ticksPerInch)) * f;
            double power = pid + ff;
            lift.setPower(power * .8);
            lift2.setPower(power * .8);
            return true;
        }
    }
    public Action liftDown() {
        return new LiftDown();
    }
    public class LiftMid implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            switch (liftState) {
                case START:
                    claw.setPosition(specClosed);
                    liftState = LiftState.LIFT;
                    break;
                case LIFT:
                    target = 600;
                    deliveryS.setPosition(backSpec);
                    if (Math.abs(lift.getCurrentPosition() - target) < 20) {
                        lift.setPower(0);
                        lift2.setPower(0);
                            liftState = LiftState.START;
                            return false;
                        }
                    break;
                default:
                    liftState = LiftState.START;

            }
            controller.setPID(p, i, d);
            int curPos = lift.getCurrentPosition();
            double pid = controller.calculate(curPos, target);
            double ff = Math.cos(Math.toRadians(target / ticksPerInch)) * f;
            double power = pid + ff;
            lift.setPower(power);
            lift2.setPower(power);
            return true;
        }
    }
    public Action liftMid() {
        return new LiftMid();
    }
    public class Pickup implements Action {
        @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                    switch (liftState) {
                        case START:
                            liftState = LiftState.DOWN;
                            deliveryTimer.reset();
                            break;
                        case DOWN:
                            deliveryS.setPosition(backSpec);
                            if (deliveryTimer.seconds() >= .5) {
                                claw.setPosition(open);
                                liftState = LiftState.START;
                                return false;
                            }
                            break;

                        default:
                            liftState = LiftState.START;
                    }
                    return true;
                }
        }
        public Action pickup() {
        return new Pickup();
        }
    public class Spec implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            switch (liftState) {
                case START:
                    deliveryS.setPosition(frontSpec);
                    claw.setPosition(specClosed);
                    liftState = LiftState.LIFT;
                    break;
                case LIFT:
                    target = 700;
                    if (Math.abs(lift.getCurrentPosition() - target) < 30) {
                        claw.setPosition(closed);
                        deliveryS.setPosition(frontSpec);
                        liftTimer.reset();
                        liftState = LiftState.LIFTED;
                    }
                    break;
                case LIFTED:
                    if (liftTimer.seconds() >= .3) {
                        claw.setPosition(open);
                        deliveryS.setPosition(midPos);
                        liftState = LiftState.DOWN;
                    }
                    break;
                case DOWN:
                    target = -20;
                    if (Math.abs(lift.getCurrentPosition() - target) < 20) {
                        lift.setPower(-.1);
                        liftState = LiftState.START;
                        return false;
                    }
                    break;
                default:
                    liftState = LiftState.START;

            }
            controller.setPID(p, i, d);
            int curPos = lift.getCurrentPosition();
            double pid = controller.calculate(curPos, target);
            double ff = Math.cos(Math.toRadians(target / ticksPerInch)) * f;
            double power = pid + ff;
            lift.setPower(power);
            theOpMode.telemetry.addData("LiftState", liftState);
            theOpMode.telemetry.addData("target", target);
            theOpMode.telemetry.addData("Current Position", lift.getCurrentPosition());
            theOpMode.telemetry.update();
            return true;
        }
    }

    public Action spec() {
        return new Spec();
    }
    public class LiftTest implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            switch (liftState) {
                case START:
                    claw.setPosition(closed);
                    deliveryS.setPosition(midPos);
                    liftState = LiftState.LIFT;
                    break;
                case LIFT:
                    target = 600;
                    if (Math.abs(lift.getCurrentPosition() - target) < 25) {
                        deliveryS.setPosition(backSpec);
                        liftTimer.reset();
                        liftState = LiftState.LIFTED;
                    }
                    break;
                case LIFTED:
                        return false;
                default:
                    liftState = LiftState.START;

            }
            controller.setPID(p, i, d);
            int curPos = lift.getCurrentPosition();
            double pid = controller.calculate(curPos, target);
            double ff = Math.cos(Math.toRadians(target / ticksPerInch)) * f;
            double power = pid + ff;
            lift.setPower(power);
            lift2.setPower(power);
            theOpMode.telemetry.addData("LiftState", liftState);
            theOpMode.telemetry.addData("target", target);
            theOpMode.telemetry.addData("Current Position", lift.getCurrentPosition());
            theOpMode.telemetry.update();

            return true;
        }
    }

    public Action liftTest() {
        return new LiftTest();
    }









    public class LiftPickup implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            switch (liftState) {
                case START:
                    claw.setPosition(closed);
                    deliveryTimer.reset();
                    liftState = LiftState.LIFT;
                    break;
                case LIFT:
                    if (deliveryTimer.seconds() >= .1) {
                        target = 300;
                        deliveryS.setPosition(backSpec);
                        if (Math.abs(lift.getCurrentPosition() - target) < 20) {
                            lift.setPower(0);
                            lift2.setPower(0);
                            liftState = LiftState.START;
                            return false;
                        }
                    }
                        break;

                default:
                    liftState = LiftState.START;

            }
            controller.setPID(p, i, d);
            int curPos = lift.getCurrentPosition();
            double pid = controller.calculate(curPos, target);
            double ff = Math.cos(Math.toRadians(target / ticksPerInch)) * f;
            double power = pid + ff;
            lift.setPower(power);
            lift2.setPower(power);
            return true;
        }
    }
    public Action liftPickup() {
        return new LiftPickup();
    }
}



