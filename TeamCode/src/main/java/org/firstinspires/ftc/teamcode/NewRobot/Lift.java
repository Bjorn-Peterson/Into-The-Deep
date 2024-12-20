package org.firstinspires.ftc.teamcode.NewRobot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.util.ArrayList;

public class Lift {
    public enum LiftState {
        START,
        LIFT,
        LIFTED,
        DOWN
    }
    LiftState liftState = LiftState.START;
    private PIDController controller;
    public static double p = 0.02, i = 0, d = 0.002;
    public static double f = 0.2;
    public static int target;
    private final double ticksPerInch = (145.1) / (1.15 * 3.14);
    double closed = .87;
    double open = .754;
    double frontSpec = .128;
    double backSpec = .56;
    double transferPos = .125;
    double midPos = .2;

    private DcMotor lift;
    public Servo claw;
    public Servo deliveryS;
    private ElapsedTime liftTimer = new ElapsedTime();
    private OpMode theOpMode;
    double countsPerInch;



    public Lift(HardwareMap hardwareMap, OpMode opMode, double encoderTicksPerRev, double gearRatio, double wheelDiameter) {
        controller = new PIDController(p, i, d);
        countsPerInch = (encoderTicksPerRev * gearRatio) / (wheelDiameter * 3.14);
        theOpMode = opMode;
        lift = hardwareMap.get(DcMotor.class, "lift");

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        claw = hardwareMap.get(Servo.class, "claw");
        deliveryS = hardwareMap.get(Servo.class, "delivery");
    }

    public void teleLift() {

        if (theOpMode.gamepad2.right_trigger > .05) {
            lift.setPower(theOpMode.gamepad2.right_trigger);

        } else if (theOpMode.gamepad2.left_trigger > .05) {
            lift.setPower(-theOpMode.gamepad2.left_trigger);

        } else {
            lift.setPower(0);
        }
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

    //public Action liftUp() {
    //    return 4;
    //}
    public class LiftAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            switch (liftState) {
                case START:
                    deliveryS.setPosition(backSpec);
                    claw.setPosition(closed);
                    liftState = LiftState.LIFT;
                    break;
                case LIFT:
                    target = 1000;
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
}

