package org.firstinspires.ftc.teamcode.NewRobot;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.OldStuff.PIDF;

public class Lift {
    public enum LiftState {
        START,
        LIFT,
        LIFTED,
        DOWN
    }

    public enum TeleState {
        START,
        TOP,
        LOW,
        SPECIMEN,
        MANUAL
    }

    TeleState teleState = TeleState.START;
    LiftState liftState = LiftState.START;
    private PIDController controller;
    public static double p = 0.035, i = 0, d = 0.001;
    public static double f = 0.031;
    public static int target;
    private final double ticksPerInch = (145.1) / (1.15 * 3.14);
    double closed = .87;
    double open = .754;
    double frontSpec = .122;
    double backSpec = .63;
    double transferPos = .18;
    double midPos = .45;
    double specClosed = .83;


    private DcMotorEx lift;
    public Servo claw;
    public Servo deliveryS;
    public DigitalChannel liftTouch;
    private ElapsedTime liftTimer = new ElapsedTime();
    private ElapsedTime deliveryTimer = new ElapsedTime();
    private OpMode theOpMode;
    double countsPerInch;
    DigitalChannel dBeam;

    double lowerMid = .35;


    public Lift(HardwareMap hardwareMap, OpMode opMode, double encoderTicksPerRev, double gearRatio, double wheelDiameter) {
        controller = new PIDController(p, i, d);
        countsPerInch = (encoderTicksPerRev * gearRatio) / (wheelDiameter * 3.14);
        theOpMode = opMode;
        lift = hardwareMap.get(DcMotorEx.class, "lift");

        lift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
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
                if (theOpMode.gamepad1.dpad_left) {
                    teleState = TeleState.SPECIMEN;
                }
                if (theOpMode.gamepad1.right_trigger > 0.1 || theOpMode.gamepad1.left_trigger > 0.1) {
                    teleState = TeleState.MANUAL;
                }

                break;
            case SPECIMEN:
                target = 800;
                controller.setPID(p, i, d);
                int curPos = lift.getCurrentPosition();
                double pid = controller.calculate(curPos, target);
                double ff = Math.cos(Math.toRadians(target)) * f;
                double power = pid + ff;
                lift.setPower(power);
                if (theOpMode.gamepad1.right_trigger > 0.1 || theOpMode.gamepad1.left_trigger > 0.1) {
                    teleState = TeleState.MANUAL;
                }

                break;
            case MANUAL:
                if (theOpMode.gamepad1.right_trigger > 0.1) {
                    lift.setPower(theOpMode.gamepad1.right_trigger);
                } else if (theOpMode.gamepad1.left_trigger > 0.1) {
                    lift.setPower(-theOpMode.gamepad1.left_trigger);

                } else if (theOpMode.gamepad1.dpad_left) {
                    teleState = TeleState.SPECIMEN;
                } else {
                    lift.setPower(0);
                }
                break;
            default:
                teleState = TeleState.START;
        }
        theOpMode.telemetry.addData("target", target);
        theOpMode.telemetry.addData("Current Position", lift.getCurrentPosition());
        theOpMode.telemetry.addData("Current State", teleState);
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

            }
            else if (theOpMode.gamepad1.left_trigger > .1) {
                lift.setPower(-theOpMode.gamepad1.left_trigger);

            } else {
                lift.setPower(0);
            }
                if (!liftTouch.getState()) {
                    teleState = TeleState.LOW;
                }
                break;
            case LOW:
                lift.setPower(0);
                if (theOpMode.gamepad1.right_trigger > .1) {
                    lift.setPower(theOpMode.gamepad1.right_trigger);
                    teleState = TeleState.MANUAL;
                }
                break;
            default: teleState = TeleState.START;
        }

    }

    public class LiftAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            switch (liftState) {
                case START:
                    claw.setPosition(closed);
                    liftState = LiftState.LIFT;
                    break;
                case LIFT:
                    target = 1345;
                    if (Math.abs(lift.getCurrentPosition() - target) < 200) {
                        deliveryS.setPosition(backSpec);
                        claw.setPosition(open);
                        liftTimer.reset();
                        liftState = LiftState.LIFTED;
                    }
                    break;
                case LIFTED:
                    if (liftTimer.seconds() >= .3) {
                        deliveryS.setPosition(midPos);

                        if (liftTimer.seconds() >= .42) {
                            liftState = LiftState.DOWN;
                        }
                    }
                    break;
                case DOWN:
                    target = -30;
                    if (Math.abs(lift.getCurrentPosition() - target) < 15) {
                        lift.setPower(-0.1);
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

    public Action liftAction() {
        return new LiftAction();
    }

    public class SpecDeliver implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            switch (liftState) {
                case START:
                    claw.setPosition(closed);
                    liftState = LiftState.LIFT;
                    break;
                case LIFT:
                    target = 1000;
                    if (Math.abs(lift.getCurrentPosition() - target) < 30) {
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
                    target = 1345;
                    if (Math.abs(lift.getCurrentPosition() - target) < 200) {
                        deliveryS.setPosition(backSpec);
                        claw.setPosition(open);
                        liftTimer.reset();
                        liftState = LiftState.LIFTED;
                    }
                    break;
                case LIFTED:
                    if (liftTimer.seconds() >= .24) {
                        deliveryS.setPosition(midPos);

                        if (liftTimer.seconds() >= .28) {
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
                    deliveryS.setPosition(midPos);
                    liftState = LiftState.DOWN;
                    break;
                case DOWN:
                    target = -30;
                    if (Math.abs(lift.getCurrentPosition() - target) < 15) {
                        lift.setPower(-0.1);
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
                    target = 860;
                    deliveryS.setPosition(frontSpec);
                    if (Math.abs(lift.getCurrentPosition() - target) < 20) {
                            lift.setPower(0);
                            claw.setPosition(closed);
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
                            target = -30;
                            deliveryS.setPosition(backSpec);
                            if (deliveryTimer.seconds() >= .2) {
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
                    target = 1000;
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
}



