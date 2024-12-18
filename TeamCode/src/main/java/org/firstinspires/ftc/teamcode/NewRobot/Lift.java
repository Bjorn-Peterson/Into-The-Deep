package org.firstinspires.ftc.teamcode.NewRobot;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.util.ArrayList;

public class Lift {
    public enum LiftState {
        START,
        MANUAL,
        AUTO
    }
    private DcMotor lift;
    private DistanceSensor sensorRange;
    private ElapsedTime runtime = new ElapsedTime();
    private OpMode theOpMode;
    double countsPerInch;
    private double kp = 0.1;
    private double ki = 0.01;
    private double kd = 0.001;

    // Variables for PID control
    private double integral = 0;
    private double previousError = 0;
    // private TouchSensor sensorTouch;


    public Lift(HardwareMap hardwareMap, OpMode opMode, double encoderTicksPerRev, double gearRatio, double wheelDiameter) {
        countsPerInch = (encoderTicksPerRev * gearRatio) / (wheelDiameter * 3.14);
        theOpMode = opMode;
        lift = hardwareMap.get(DcMotor.class, "lift");
        lift.setDirection(DcMotor.Direction.REVERSE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // sensorTouch = hardwareMap.get(TouchSensor.class, "touchSensor");
    }

    public void teleLift() {

        if (theOpMode.gamepad2.right_trigger > .05) {
            lift.setPower(-theOpMode.gamepad2.right_trigger);

        } else if (theOpMode.gamepad2.left_trigger > .05) {
            lift.setPower(theOpMode.gamepad2.left_trigger);

        } else {
            lift.setPower(0);
        }
    }

    public void soloControls() {

        if (theOpMode.gamepad1.right_trigger > .05) {
            lift.setPower(-theOpMode.gamepad1.right_trigger);

        } else if (theOpMode.gamepad1.left_trigger > .05) {
            lift.setPower(theOpMode.gamepad1.left_trigger);

        } else {
            lift.setPower(0);
        }
    }

    //public Action liftUp() {
    //    return 4;
    //}
}

