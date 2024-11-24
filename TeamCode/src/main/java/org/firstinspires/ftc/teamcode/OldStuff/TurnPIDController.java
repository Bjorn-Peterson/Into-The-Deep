package org.firstinspires.ftc.teamcode.OldStuff;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

// Single use per object
public class TurnPIDController {
    private double kP, kI, kD;
    private ElapsedTime timer = new ElapsedTime();
    private double targetPos;
    private double lastError = 0;
    private double accumulatedError = 0;
    private double lastTime = -1;
    private double lastSlope = 0;
    private double integral = 0;
    private double derivative = 0;
    private double dT = 0;
    private double masxPower = 0;
    int extended;
    int retracted;
    int mid;
    double collect;
    double transfer;
    DigitalChannel cBeam;



    DcMotorEx extend;
    DcMotorEx collection;
    public Servo rCollection;
    public Servo lCollection;
    public Servo claw;
    public Servo deliveryS;
    double countsPerInch;
    private OpMode theOpMode;

    public TurnPIDController(HardwareMap hardwareMap, OpMode opMode, double p, double i, double d, double encoderTicksPerRev, double gearRatio, double wheelDiameter) {
        kP = p;
        kI = i;
        kD = d;


        theOpMode = opMode;
        countsPerInch = (encoderTicksPerRev * gearRatio) / (wheelDiameter * 3.14);
        extend = hardwareMap.get(DcMotorEx.class, "extend");
        extend.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        extend.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        extend.setDirection(DcMotorSimple.Direction.REVERSE);
        lCollection = hardwareMap.get(Servo.class, "lCollection");
        rCollection = hardwareMap.get(Servo.class, "rCollection");
        claw = hardwareMap.get(Servo.class, "claw");
        deliveryS = hardwareMap.get(Servo.class, "delivery");
        collection = hardwareMap.get(DcMotorEx.class, "collection");

        cBeam = hardwareMap.get(DigitalChannel.class, "beam");
        cBeam.setMode(DigitalChannel.Mode.INPUT);
    }

    public double update() {


        // P
        double error = targetPos - extend.getCurrentPosition();
        error %= 360;
        error += 360;
        error %= 360;
        if (error > 180) {
            error -= 360;
        }
        extended = (int) (error);
        retracted = (int) (error);
        mid = (int) (error);


        dT = (timer.milliseconds() - lastTime);
        integral = integral + error * dT;
        derivative = ((error - lastError) / dT);
        double motorPower = (kP * error + kI *integral + kD * derivative);
        motorPower = Math.max(-1, Math.min(motorPower, 1));
        // I

        // D
        double slope = 0;
        if (lastTime > 0) {
            slope = (error - lastError) / (timer.milliseconds() - lastTime);
        }

        lastSlope = slope;
        lastError = error;
        lastTime = timer.milliseconds();
        if (theOpMode.gamepad1.x) {
            targetPos = 100;
            extend.setTargetPosition(mid);
            extend.setPower(motorPower);
            theOpMode.telemetry.addData("Target Position", mid);
            theOpMode.telemetry.addData("Current Position", extend.getCurrentPosition());
            theOpMode.telemetry.update();
        }
        if (theOpMode.gamepad1.y) {
            lCollection.setPosition(.5);
        }
        if (theOpMode.gamepad1.b) {
            targetPos = 1;
            extend.setTargetPosition(retracted);
            extend.setPower(motorPower);
            theOpMode.telemetry.addData("Target Position", retracted);
            theOpMode.telemetry.addData("Current Position", extend.getCurrentPosition());
            theOpMode.telemetry.update();
        }





        return motorPower;

    }

}
