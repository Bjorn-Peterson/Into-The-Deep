package org.firstinspires.ftc.teamcode.OldStuff;

import java.lang.Math;
import java.sql.Time;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class PDFL{
   /* public enum ExtendState {
        START,
        RETRACT,
        MID,
        EXTEND,
        EXTENDED

    }

    public enum DeliveryState {
        START,
        COLLECT,
        SPECIMEN,
        SAMPLE
    }
    ExtendState extendState = ExtendState.START;

    DeliveryState deliveryState = DeliveryState.START;

    */

    private double kP, kD, kF, kL;

    private double deadzone = 5;

    private double homedConstant;

    private boolean homed = false;
    DcMotorEx extend;
    DcMotorEx collection;
    public Servo rCollection;
    public Servo lCollection;
    public Servo claw;
    public Servo deliveryS;
    double countsPerInch;
   // private Timer timer = new Timer();
    ElapsedTime timer = new ElapsedTime();

    private RingBuffer<Double> timeBuffer = new RingBuffer<Double>(3, 0.0);
    private RingBuffer<Double> errorBuffer = new RingBuffer<Double>(3, 0.0);
    private OpMode theOpMode;



     int extended;
     int retracted;
     int mid;
     double collect;
     double transfer;
     DigitalChannel cBeam;

    public PDFL(double kP, double kD, double kF, double kL, HardwareMap hardwareMap, OpMode opMode, double encoderTicksPerRev, double gearRatio, double wheelDiameter){
        theOpMode = opMode;
        countsPerInch = (encoderTicksPerRev * gearRatio) / (wheelDiameter * 3.14);
        extend = hardwareMap.get(DcMotorEx.class, "extend");
        extend.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        extend.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        extend.setDirection(DcMotorSimple.Direction.REVERSE);
        lCollection = hardwareMap.get(Servo.class, "lCollection");
        rCollection = hardwareMap.get(Servo.class, "rCollection");
        rCollection.setDirection(Servo.Direction.REVERSE);
        claw = hardwareMap.get(Servo.class, "claw");
        deliveryS = hardwareMap.get(Servo.class, "delivery");
        collection = hardwareMap.get(DcMotorEx.class, "collection");
        //collection.setDirection(DcMotorSimple.Direction.REVERSE);

        cBeam = hardwareMap.get(DigitalChannel.class, "beam");
        cBeam.setMode(DigitalChannel.Mode.INPUT);





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
        extended = (int) (error + 70);
        retracted = (int) (error + 1);
        mid = (int) (error + 30);

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

        double response = (p + d + f + l);

        if (Math.abs(error) < deadzone){
            //same response but without lower limit
            response = (p + d + f);
        }
        if (theOpMode.gamepad1.a) {
           deliveryS.setPosition(.8);
        }
        else if (theOpMode.gamepad1.b) {
            deliveryS.setPosition(.5);
        }
/*
        if (theOpMode.gamepad1.x) {
            extend.setPower(response);
            extend.setTargetPosition(mid);
            theOpMode.telemetry.addData("Target Position", mid);
            theOpMode.telemetry.addData("Current Position", extend.getCurrentPosition());
            theOpMode.telemetry.update();
        }
        if (theOpMode.gamepad1.y) {
            lCollection.setPosition(.5);
        }
        if (theOpMode.gamepad1.a) {
            rCollection.setPosition(.5);
        }
        if (theOpMode.gamepad1.b) {
            extend.setPower(response);
            extend.setTargetPosition(retracted);
            theOpMode.telemetry.addData("Target Position", retracted);
            theOpMode.telemetry.addData("Current Position", extend.getCurrentPosition());
            theOpMode.telemetry.update();
        }

 */

        /*
        switch (extendState) {
            case START:

                if (theOpMode.gamepad1.x) {
                    extend.setTargetPosition(extended);
                    extendState = ExtendState.EXTEND;
                    collection.setPower(.8);
                } else if (theOpMode.gamepad1.y) {
                    rCollection.setPosition(collect);
                    lCollection.setPosition(collect);
                    extend.setTargetPosition(mid);
                    extendState = ExtendState.MID;
                    collection.setPower(.8);
                }
                break;
            case EXTEND:
                if (Math.abs(extend.getCurrentPosition() * countsPerInch) - extended < 10) {
                    rCollection.setPosition(collect);
                    lCollection.setPosition(collect);
                    extendState = ExtendState.EXTENDED;
                }
                break;
            case EXTENDED:
            case MID:
                if (!cBeam.getState()) {
                    collection.setPower(0);
                    rCollection.setPosition(transfer);
                    lCollection.setPosition(transfer);
                    extendState = ExtendState.RETRACT;
                    extend.setTargetPosition(retracted);
                    theOpMode.telemetry.addData("Beam", "Broken");

                }
                break;
            default:
                extendState = ExtendState.START;
        }
                if (theOpMode.gamepad1.a && extendState !=  ExtendState.START) {
                    extendState = ExtendState.START;
                }


         */
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