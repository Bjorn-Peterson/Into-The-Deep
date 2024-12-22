package org.firstinspires.ftc.teamcode.NewRobot;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
        TOP,
        LOW,
        SPECIMEN,
        MANUAL
    }

    TeleState teleState = TeleState.START;
    LiftState liftState = LiftState.START;
    private PIDController controller;
    public static double p = 0.033, i = 0, d = 0.001;
    public static double f = 0.01;
    public static int target;
    private final double ticksPerInch = (145.1) / (1.15 * 3.14);
    double closed = .87;
    double open = .754;
    double frontSpec = .12;
    double backSpec = .59;
    double transferPos = .18;
    double midPos = .3;
    double specClosed = .85;


    private DcMotorEx lift;
    public Servo claw;
    public Servo deliveryS;
    private ElapsedTime liftTimer = new ElapsedTime();
    private OpMode theOpMode;
    double countsPerInch;


    public Lift(HardwareMap hardwareMap, OpMode opMode, double encoderTicksPerRev, double gearRatio, double wheelDiameter) {
        controller = new PIDController(p, i, d);
        countsPerInch = (encoderTicksPerRev * gearRatio) / (wheelDiameter * 3.14);
        theOpMode = opMode;
        lift = hardwareMap.get(DcMotorEx.class, "lift");

        lift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        claw = hardwareMap.get(Servo.class, "claw");
        deliveryS = hardwareMap.get(Servo.class, "delivery");
    }

    public void teleLift() {

        switch (teleState) {
            case START:
                if (theOpMode.gamepad2.right_trigger > .05 || theOpMode.gamepad2.left_trigger > .05) {
                    teleState = TeleState.MANUAL;
                }
                if (theOpMode.gamepad1.dpad_right) {
                    teleState = TeleState.TOP;
                }
                break;
            case MANUAL:
                if (theOpMode.gamepad2.right_trigger > .05) {
                    lift.setPower(theOpMode.gamepad2.right_trigger);
                }

                else if (theOpMode.gamepad2.left_trigger > .05) {
                    lift.setPower(-theOpMode.gamepad2.left_trigger);
                }
                else {
                    lift.setPower(0);
                }
                break;
            case TOP:
                target = 1600;
            default: teleState = TeleState.START;
        }


        switch (liftState) {
            case START:
                deliveryS.setPosition(backSpec);
                claw.setPosition(closed);
                if (theOpMode.gamepad2.x) {
                    liftState = LiftState.LIFT;
                }
                break;
            case LIFT:
                target = 1600;
                if (Math.abs(lift.getCurrentPosition() - target) < 20) {
                    claw.setPosition(open);
                    liftTimer.reset();
                    liftState = LiftState.LIFTED;
                }
                break;
            case LIFTED:
                if (liftTimer.seconds() >= .5) {
                    target = 40;
                    deliveryS.setPosition(midPos);
                    liftState = LiftState.DOWN;

                }
                break;
            case DOWN:
                if (Math.abs(lift.getCurrentPosition() - target) < 20) {
                    liftState = LiftState.START;
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
    }

    public void soloControls() {

        if (theOpMode.gamepad1.right_trigger > .05) {
            lift.setPower(theOpMode.gamepad1.right_trigger);

        } else if (theOpMode.gamepad1.left_trigger > .05) {
            lift.setPower(-theOpMode.gamepad1.left_trigger);

        } else {
            lift.setPower(0);
        }
    }

    public class LiftAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            switch (liftState) {
                case START:
                    lift.setPower(0);
                    deliveryS.setPosition(backSpec);
                    claw.setPosition(closed);
                    liftState = LiftState.LIFT;
                    break;
                case LIFT:
                    target = 1345;
                    if (Math.abs(lift.getCurrentPosition() - target) < 30) {
                        claw.setPosition(open);
                        liftTimer.reset();
                        liftState = LiftState.LIFTED;
                    }
                    break;
                case LIFTED:
                    if (liftTimer.seconds() >= .5) {
                        deliveryS.setPosition(midPos);
                        liftState = LiftState.DOWN;

                    }
                    break;
                case DOWN:
                    target = 20;
                    if (Math.abs(lift.getCurrentPosition() - target) < 20) {
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
                    claw.setPosition(specClosed);
                    liftState = LiftState.LIFT;
                    break;
                case LIFT:
                    target = 900;
                    if (Math.abs(lift.getCurrentPosition() - target) < 100) {
                        deliveryS.setPosition(frontSpec);
                        liftTimer.reset();
                        liftState = LiftState.LIFTED;
                    }
                    break;
                case LIFTED:
                    if (liftTimer.seconds() >= .8) {
                        claw.setPosition(open);
                        deliveryS.setPosition(midPos);
                        liftState = LiftState.DOWN;
                    }
                    break;
                case DOWN:
                    target = 40;
                    if (Math.abs(lift.getCurrentPosition() - target) < 20) {
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
    }


