package org.firstinspires.ftc.teamcode.OldStuff;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class OldLift {
    public enum LiftState {
        START,
        EXTEND,
        EXTENDED,
        RETRACT
    }
    LiftState liftState = LiftState.START;

    public DcMotor leftMotor;
    public DcMotor rightMotor;
    Servo rDelivery;
    private ElapsedTime runtime = new ElapsedTime();
    private OpMode theOpMode;
    double countsPerInch;
    private PIDController controller;
    public static double p = 0.01, i = 0, d = 0.0001;
    public static double f = 0;
    public static int target;
    private final double ticksPerInch = (145.1) / (1.15 * 3.14);
    ElapsedTime liftTimer = new ElapsedTime();
    int retracted = 20;
    boolean liftDone;
    public boolean liftStart;


    public OldLift(HardwareMap hardwareMap, OpMode opMode, double encoderTicksPerRev, double gearRatio, double wheelDiameter) {
        countsPerInch = (encoderTicksPerRev * gearRatio) / (wheelDiameter * 3.14);
        controller = new PIDController(p, i, d);

        theOpMode = opMode;
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        rDelivery = hardwareMap.get(Servo.class, "rDelivery");

        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void teleLift() {

        if (theOpMode.gamepad2.right_trigger > .05) {
            leftMotor.setPower(theOpMode.gamepad2.right_trigger);
            rightMotor.setPower(theOpMode.gamepad2.right_trigger);

        }
        else if (theOpMode.gamepad2.left_trigger > .05) {
            leftMotor.setPower(-theOpMode.gamepad2.left_trigger);
            rightMotor.setPower(-theOpMode.gamepad2.left_trigger);

        }
        else {
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }
    }
    public void soloControls(){

            if (theOpMode.gamepad1.right_trigger > .05) {
                leftMotor.setPower(theOpMode.gamepad1.right_trigger);
                rightMotor.setPower(theOpMode.gamepad1.right_trigger);

            }
            else if (theOpMode.gamepad1.left_trigger > .05) {
                leftMotor.setPower(-theOpMode.gamepad1.left_trigger);
                rightMotor.setPower(-theOpMode.gamepad1.left_trigger);

            }
            else {
                leftMotor.setPower(0);
                rightMotor.setPower(0);
            }
        }


    public void liftAuto(double power, int target, double timeoutS) {
        runtime.reset();
        leftMotor.setPower(power);
        rightMotor.setPower(power);
        leftMotor.setTargetPosition(target);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (((LinearOpMode) theOpMode).opModeIsActive() && runtime.seconds() < timeoutS && leftMotor.isBusy()) {

            theOpMode.telemetry.addData("target", target);
            theOpMode.telemetry.addData("Current Position", leftMotor.getCurrentPosition());
            theOpMode.telemetry.update();

        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    public void auto() {
        liftDone = false;
        while (((LinearOpMode) theOpMode).opModeIsActive() && !liftDone) {
            switch (liftState) {
                case START:
                    if (liftStart) {
                        liftState = LiftState.EXTEND;
                        rDelivery.setPosition(.36);
                    }
                    break;
                case EXTEND:
                    target = 1700;
                    if (Math.abs(leftMotor.getCurrentPosition() - target) < 320) {
                        rDelivery.setPosition(.75);
                        liftTimer.reset();
                        liftState = LiftState.EXTENDED;
                    }
                    break;
                case EXTENDED:
                    if (liftTimer.seconds() >= 1.4) {
                        liftState = LiftState.RETRACT;
                        rDelivery.setPosition(.36);
                    }
                    break;
                case RETRACT:
                    target = retracted;
                    if (Math.abs(leftMotor.getCurrentPosition() - target) < 20) {
                        liftDone = true;
                        liftStart = false;
                        liftState = LiftState.START;
                    }
                    break;
                default:
                    liftState = LiftState.START;
            }


            controller.setPID(p, i, d);
            int curPos = leftMotor.getCurrentPosition();
            double pid = controller.calculate(curPos, target);
            double ff = Math.cos(Math.toRadians(target / ticksPerInch)) * f;
            double power = pid + ff;
            leftMotor.setPower(power);
            rightMotor.setPower(power);
            theOpMode.telemetry.addData("pos", curPos);
            theOpMode.telemetry.addData("target", target);
            theOpMode.telemetry.addData("power", power);
            theOpMode.telemetry.addData("Lift State", liftState);
            theOpMode.telemetry.update();
        }
    }
}
