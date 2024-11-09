package org.firstinspires.ftc.teamcode.OldStuff;

import java.lang.Math;
import java.sql.Time;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class PDFL{

    private double[] liftHeights = {16, 25, 43};

    private double kP, kD, kF, kL;

    private double deadzone = 5;

    private double homedConstant;

    private boolean homed = false;
    DcMotorEx collection;
    double countsPerInch;
  //  private Timer timer = new Timer();
    ElapsedTime timer = new ElapsedTime();

    private RingBuffer<Double> timeBuffer = new RingBuffer<Double>(3, 0.0);
    private RingBuffer<Double> errorBuffer = new RingBuffer<Double>(3, 0.0);
    private OpMode theOpMode;

    public PDFL(double kP, double kD, double kF, double kL, HardwareMap hardwareMap, OpMode opMode, double encoderTicksPerRev, double gearRatio, double wheelDiameter){
        theOpMode = opMode;
        countsPerInch = (encoderTicksPerRev * gearRatio) / (wheelDiameter * 3.14);
        collection = hardwareMap.get(DcMotorEx.class, "collection");


        this.kP = kP;
        this.kD = kD;
        this.kF = kF;
        this.kL = kL;
    }

    public void updateConstants(double kP, double kD, double kF, double kL){
        this.kP = kP;
        this.kD = kD;
        this.kF = kF;
        this.kL = kL;
    }

    public void setDeadzone(double deadzone){
        this.deadzone = deadzone;
    }

    public void setHomed(boolean homed){
        this.homed = homed;
    }

    public void setHomedConstant(double constant){
        homedConstant = constant;
    }

    public void reset(){
        timeBuffer.fill(0.0);
        errorBuffer.fill(0.0);
        timer.reset();
    }


    public double run(double error){

        if (homed){
            return homedConstant;
        }

        double time = timer.time();

        double previous_time = timeBuffer.getValue(time);
        double previous_error = errorBuffer.getValue(error);

        double delta_time = time - previous_time;
        double delta_error = error - previous_error;

        //If the PDFL hasn't been updated, reset it
        if (delta_time > 200){
            reset();
            return run(error);
        }

        double p = pComponent(error);
        double d = dComponent(delta_error, delta_time);
        double f = fComponenet();
        double l = lComponent(error);

        double response = (p + d + f + l) * countsPerInch;

        if (Math.abs(error) < deadzone){
            //same response but without lower limit
            response = (p + d + f) * countsPerInch;
        }

        return response;
    }

    private double pComponent(double error){

        double response = kP * error;

        return response;
    }

    private double dComponent(double delta_error, double delta_time){

        double derivative = delta_error / delta_time;

        double response = derivative * kD;

        return response;
    }

    private double fComponenet(){

        double response = kF;

        return response;
    }

    private double lComponent(double error){

        double direction = Math.signum(error);

        double response = direction * kL;

        return response;
    }


}