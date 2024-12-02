package org.firstinspires.ftc.teamcode.OldStuff;


import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.NewRobot.Delivery;


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
    int extended = 680;
    int retracted = 0;
    int mid = 400;
    double collect = .65;
    double transfer = .51;
    double xHeight = .46;
    double closed = .85;
    double open = .7;
    double frontSpec = .05;
    double backSpec = .54;
    double transferPos = .053;
    DigitalChannel cBeam;
    DigitalChannel dBeam;
    DigitalChannel lSwitch;

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
        dBeam = hardwareMap.get(DigitalChannel.class, "dBeam");
        dBeam.setMode(DigitalChannel.Mode.INPUT);
        lSwitch = hardwareMap.get(DigitalChannel.class, "lSwitch");
        lSwitch.setMode(DigitalChannel.Mode.INPUT);
    }

    public void tele() {
        if (theOpMode.gamepad1.left_bumper) {
            claw.setPosition(open);
        }
        else if (theOpMode.gamepad1.right_bumper) {
            claw.setPosition(closed);
        }
        if (theOpMode.gamepad1.dpad_up) {
            deliveryState = DeliveryState.DELIVER;
            deliveryS.setPosition(backSpec);
        }
        else if (theOpMode.gamepad1.dpad_down) {
            deliveryState = DeliveryState.COLLECT;
            deliveryS.setPosition(frontSpec);
        }
        switch (extendState) {
            //Fully Retracted in transfer position
            case START:
                collection.setPower(0);
                rCollection.setPosition(transfer);
                lCollection.setPosition(transfer);

                // Start extending, turn on collection
                if (theOpMode.gamepad1.x) {
                    target = extended;
                    rCollection.setPosition(xHeight);
                    lCollection.setPosition(xHeight);
                    // extend.setTargetPosition(extended);
                    extendState = ExtendState.EXTEND;
                    collection.setPower(.8);
                }
                if (theOpMode.gamepad1.y) {
                    target = mid;
                    rCollection.setPosition(collect);
                    lCollection.setPosition(collect);
                    extendState = ExtendState.MID;
                    collection.setPower(.8);
                }
                if (theOpMode.gamepad1.dpad_up) {
                    deliveryS.setPosition(backSpec);
                }
                else if (theOpMode.gamepad1.dpad_down) {
                    deliveryS.setPosition(frontSpec);
                }
                else if (theOpMode.gamepad1.dpad_left) {
                    deliveryS.setPosition(transferPos);
                }
                break;
                // Rotate to collecting position
            case EXTEND:
                if (Math.abs(extend.getCurrentPosition() - extended) < 30 && theOpMode.gamepad1.x) {
                    rCollection.setPosition(collect);
                    lCollection.setPosition(collect);
                    extendState = ExtendState.EXTENDED;
                }
                break;
            case EXTENDED:
            case MID:
                // If we collect a specimen, retract extension, rotate to transfer position, stop collection
                if (!cBeam.getState()) {
                    deliveryS.setPosition(transferPos);
                    target = retracted;
                    collection.setPower(0);
                    rCollection.setPosition(xHeight);
                    lCollection.setPosition(xHeight);
                    extendState = ExtendState.RETRACT;
                    theOpMode.telemetry.addData("Beam", "Broken");

                }
                if (theOpMode.gamepad1.x) {
                    target = extended;
                    rCollection.setPosition(xHeight);
                    lCollection.setPosition(xHeight);
                    extendState = ExtendState.EXTEND;
                    collection.setPower(.8);
                }
                if (theOpMode.gamepad1.y) {
                    target = mid;
                    rCollection.setPosition(collect);
                    lCollection.setPosition(collect);
                    extendState = ExtendState.MID;
                    collection.setPower(.8);
                }
                break;
            case RETRACT:
                deliveryState = DeliveryState.COLLECT;
                deliveryS.setPosition(transferPos);
                if (Math.abs(extend.getCurrentPosition() - retracted) < 30) {
                    rCollection.setPosition(transfer);
                    lCollection.setPosition(transfer);
                    collection.setPower(.7);
                }
                if (!dBeam.getState()) {
                    extendState = ExtendState.START;
                    deliveryState = DeliveryState.COLLECT;
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
        double power = controller.calculate(curPos, target);
        extend.setPower(power);
        theOpMode.telemetry.addData("pos", curPos);
        theOpMode.telemetry.addData("target", target);
        theOpMode.telemetry.addData("Current State", extendState);
        theOpMode.telemetry.update();



        switch (deliveryState) {

            case COLLECT:
                    claw.setPosition(closed);
                    deliveryS.setPosition(backSpec);
                    if (theOpMode.gamepad1.left_bumper) {
                        claw.setPosition(open);
                    }




            default: deliveryState = DeliveryState.START;
            if (theOpMode.gamepad1.dpad_down && deliveryState != DeliveryState.START) {
                deliveryState = DeliveryState.START;
            }


        }

    }


}
