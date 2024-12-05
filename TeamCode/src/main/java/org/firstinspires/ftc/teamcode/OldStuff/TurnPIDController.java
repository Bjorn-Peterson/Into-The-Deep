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




    private OpMode theOpMode;

    public TurnPIDController(HardwareMap hardwareMap, OpMode opMode, double target, double p, double i, double d, double encoderTicksPerRev, double gearRatio, double wheelDiameter) {
        kP = p;
        kI = i;
        kD = d;
        targetPos = target;



        theOpMode = opMode;


    }

    public double update(double currentPos) {


        // P
        double error = targetPos - currentPos;
        error %= 360;
        error += 360;
        error %= 360;
        if (error > 180) {
            error -= 360;
        }


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

        return motorPower;

    }

}
