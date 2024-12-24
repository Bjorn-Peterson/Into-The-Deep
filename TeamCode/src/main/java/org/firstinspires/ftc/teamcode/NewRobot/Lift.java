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
    public static double p = 0.033, i = 0, d = 0.001;
    public static double f = 0.03;
    public static int target;
    private final double ticksPerInch = (145.1) / (1.15 * 3.14);
    double closed = .87;
    double open = .754;
    double frontSpec = .14;
    double backSpec = .6;
    double transferPos = .18;
    double midPos = .45;
    double specClosed = .83;


    private DcMotorEx lift;
    public Servo claw;
    public Servo deliveryS;
    private ElapsedTime liftTimer = new ElapsedTime();
    private ElapsedTime deliveryTimer = new ElapsedTime();
    private OpMode theOpMode;
    double countsPerInch;
    DigitalChannel dBeam;
    PIDF pidf;


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
    }

    public void teleLift() {

        switch (teleState) {
            case START:
            if (theOpMode.gamepad1.dpad_left && !dBeam.getState()) {
                teleState = TeleState.SPECIMEN;
            }
            if (theOpMode.gamepad1.right_trigger > 0 || theOpMode.gamepad1.left_trigger > 0) {
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
                if (theOpMode.gamepad1.right_trigger > 0 || theOpMode.gamepad1.left_trigger > 0) {
                    teleState = TeleState.MANUAL;
                }

                break;
            case MANUAL:
                if (theOpMode.gamepad1.right_trigger > 0) {
                    lift.setPower(theOpMode.gamepad1.right_trigger);
                } else if (theOpMode.gamepad1.left_trigger > 0) {
                    lift.setPower(-theOpMode.gamepad1.left_trigger);

                }
                else if (theOpMode.gamepad1.dpad_left) {
                    teleState = TeleState.SPECIMEN;
                }
                else {
                    lift.setPower(0);
                }
                break;
            default: teleState = TeleState.START;
            }
        theOpMode.telemetry.addData("target", target);
        theOpMode.telemetry.addData("Current Position", lift.getCurrentPosition());
        theOpMode.telemetry.addData("Current State", teleState);
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
                        lift.setPower(-0.15);
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
                    target = -20;
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


