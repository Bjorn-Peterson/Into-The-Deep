package org.firstinspires.ftc.teamcode.OldStuff;
//Sensor

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.NewRobot.Delivery;


public class PDFL {
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
        DELIVER,
        SPECIMEN
    }

    ExtendState extendState = ExtendState.START;

    DeliveryState deliveryState = DeliveryState.START;
    private PIDController controller;
    public static double p = 0.01, i = 0, d = 0.0001;
    public static int target;


    private OpMode theOpMode;
    DcMotorEx extend;
    DcMotorEx collection;
    public Servo rCollection;
    public Servo lCollection;
    public Servo claw;
    public Servo deliveryS;
    int extended = 630;
    int retracted = 0;
    int mid = 400;
    double collect = .645;
    double transfer = .57;
    double xHeight = .46;
    double closed = .85;
    double open = .7;
    double frontSpec = .05;
    double backSpec = .54;
    double transferPos = .122;
    double midPos = .3;
    DigitalChannel cBeam;
    DigitalChannel dBeam;
    DigitalChannel lSwitch;

    public PDFL(HardwareMap hardwareMap, OpMode opMode) {
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
        dBeam = hardwareMap.get(DigitalChannel.class, "dBeam");
        dBeam.setMode(DigitalChannel.Mode.INPUT);
        lSwitch = hardwareMap.get(DigitalChannel.class, "lSwitch");
        lSwitch.setMode(DigitalChannel.Mode.INPUT);
    }

    public void auto(int target) {
        while (((LinearOpMode) theOpMode).opModeIsActive()) {
        switch (extendState) {

                //Fully Retracted in transfer position
                case START:
                    collection.setPower(0);
                    extendState = ExtendState.EXTEND;

                    rCollection.setPosition(collect);
                    lCollection.setPosition(collect);

                    break;
                // Rotate to collecting position
                case EXTEND:
                    deliveryS.setPosition(midPos);
                    if (Math.abs(extend.getCurrentPosition() - target) < 20) {
                        collection.setPower(.8);
                        extendState = ExtendState.EXTENDED;
                    }
                    break;
                case EXTENDED:
                    // If we collect a specimen, retract extension, rotate to transfer position, stop collection
                    if (!cBeam.getState()) {
                        target = retracted;
                        deliveryS.setPosition(transferPos);
                        //target = retracted;
                        collection.setPower(0);
                        extendState = ExtendState.RETRACT;
                        theOpMode.telemetry.addData("Beam", "Broken");

                    }
                    break;
                case RETRACT:
                    deliveryS.setPosition(transferPos);
                    claw.setPosition(open);
                    if (Math.abs(extend.getCurrentPosition() - retracted) < 30 && !lSwitch.getState()) {
                        rCollection.setPosition(transfer);
                        lCollection.setPosition(transfer);
                        collection.setPower(.5);

                    }
                    if (!dBeam.getState()) {
                        extendState = ExtendState.START;
                        deliveryState = DeliveryState.COLLECT;
                    }


                    break;

                default:
                    extendState = ExtendState.START;
            }
            controller.setPID(p, i, d);
            int curPos = extend.getCurrentPosition();
            double power = controller.calculate(curPos, target);
            extend.setPower(power);
            theOpMode.telemetry.addData("pos", curPos);
            theOpMode.telemetry.addData("target", target);
            theOpMode.telemetry.addData("Current State", extendState);
            theOpMode.telemetry.update();

        }


    }
}
