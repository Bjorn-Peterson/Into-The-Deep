package org.firstinspires.ftc.teamcode.OldStuff;

import android.graphics.Path;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDF {
    public enum ExtendState {
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
    private PIDController controller;
    public static double p = 0.01, i = 0, d = 0.0001;
    public static int target;


    private ElapsedTime runtime = new ElapsedTime();
    private OpMode theOpMode;
    DcMotorEx extend;
    DcMotorEx collection;
    public Servo rCollection;
    public Servo lCollection;
    public Servo claw;
    public Servo deliveryS;
    double countsPerInch;
    int extended = 650;
    int retracted = -10;
    int mid = 400;
    double collect = .63;
    double transfer = .54;
    double closed = .85;
    double open = .75;
    double frontSpec = .5;
    double backSpec = .9;
    double transferPos = .12;
    DigitalChannel cBeam;
    ElapsedTime rotateTimer = new ElapsedTime();
    double rotateTime;
    double transferTimer = .8;

    public PIDF(HardwareMap hardwareMap, OpMode opMode) {
        theOpMode = opMode;
        controller = new PIDController(p, i, d);
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
    }

    public void tele() {
        switch (extendState) {
            case START:
                rCollection.setPosition(transfer);
                lCollection.setPosition(transfer);
                deliveryS.setPosition(transferPos);


                if (theOpMode.gamepad1.x) {
                    target = extended;
                    rCollection.setPosition(transfer);
                    lCollection.setPosition(transfer);
                    // extend.setTargetPosition(extended);
                    extendState = ExtendState.EXTEND;
                    collection.setPower(.8);
                }
                if (theOpMode.gamepad1.y) {
                    target = mid;
                    rCollection.setPosition(collect);
                    lCollection.setPosition(collect);
                    //extend.setTargetPosition(mid);
                    extendState = ExtendState.MID;
                    collection.setPower(.8);
                }
                break;
            case EXTEND:
                if (Math.abs(extend.getCurrentPosition() - extended) < 30) {
                    rCollection.setPosition(collect);
                    lCollection.setPosition(collect);
                    extendState = ExtendState.EXTENDED;
                }
                break;
            case EXTENDED:
            case MID:
                if (!cBeam.getState()) {
                    target = retracted;
                    collection.setPower(0);
                    rCollection.setPosition(transfer);
                    lCollection.setPosition(transfer);
                    extendState = ExtendState.RETRACT;
                    //extend.setTargetPosition(retracted);
                    theOpMode.telemetry.addData("Beam", "Broken");

                }
                if (theOpMode.gamepad1.x) {
                    target = extended;
                    rCollection.setPosition(transfer);
                    lCollection.setPosition(transfer);
                    // extend.setTargetPosition(extended);
                    extendState = ExtendState.EXTEND;
                    collection.setPower(.8);
                }
                if (theOpMode.gamepad1.y) {
                    target = mid;
                    rCollection.setPosition(collect);
                    lCollection.setPosition(collect);
                    //extend.setTargetPosition(mid);
                    extendState = ExtendState.MID;
                    collection.setPower(.8);
                }
                break;
            case RETRACT:
                deliveryS.setPosition(transferPos);
                claw.setPosition(open);
                if (Math.abs(extend.getCurrentPosition() - retracted) < 30 && !cBeam.getState()) {
                    rotateTimer.reset();
                    rCollection.setPosition(transfer);
                    lCollection.setPosition(transfer);
                    collection.setPower(.6);
                    if (cBeam.getState() && rotateTimer.seconds() >= .5) {
                        extendState = ExtendState.START;
                        deliveryState = DeliveryState.COLLECT;
                        claw.setPosition(closed);
                        deliveryS.setPosition(frontSpec);
                    }
                }
                break;

            default:
                extendState = ExtendState.START;
        }
        if (theOpMode.gamepad1.a && extendState !=  ExtendState.START) {
            rCollection.setPosition(transfer);
            lCollection.setPosition(transfer);
            collection.setPower(0);
            target = retracted;
            extendState = ExtendState.START;
        }
        controller.setPID(p, i, d);
        int curPos = extend.getCurrentPosition();
        double pid = controller.calculate(curPos, target);
        double power = pid;
        extend.setPower(power);
        theOpMode.telemetry.addData("pos", curPos);
        theOpMode.telemetry.addData("target", target);
        theOpMode.telemetry.addData("Current State", extendState);
        theOpMode.telemetry.addData("timer",rotateTimer);
        theOpMode.telemetry.update();

    }
    public void testTele() {

    }
}
