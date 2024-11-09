package org.firstinspires.ftc.teamcode.OldStuff;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.DigitalChannel;

public class Collection3d {
    private ElapsedTime runtime = new ElapsedTime();
    Servo rCollection;
    Servo lCollection;
    Servo rDelivery;
    Servo lDelivery;
    Servo box;
    CRServo hangS;
    DcMotor collection;
    DcMotor hang;
    DigitalChannel cBeam;



    private OpMode theOpMode;

    public Collection3d(HardwareMap hardwareMap, OpMode opMode) {
        theOpMode = opMode;
        rCollection = hardwareMap.get(Servo.class, "rCollection");
        lCollection = hardwareMap.get(Servo.class, "lCollection");
        rDelivery = hardwareMap.get(Servo.class, "rDelivery");
        lDelivery = hardwareMap.get(Servo.class, "lDelivery");
        collection = hardwareMap.get(DcMotorEx.class, "collection");
        box = hardwareMap.get(Servo.class, "box");
        hang = hardwareMap.get(DcMotor.class, "hang");
        hangS = hardwareMap.get(CRServo.class, "hangS");

        collection.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lDelivery.setDirection(Servo.Direction.REVERSE);

        cBeam = hardwareMap.get(DigitalChannel.class, "beam");

        cBeam.setMode(DigitalChannel.Mode.INPUT);

    }

    public void teleopControls() {

        if(theOpMode.gamepad1.dpad_up || theOpMode.gamepad2.dpad_up) {
            rDelivery.setPosition(.4);
        } else if(theOpMode.gamepad1.dpad_down || theOpMode.gamepad2.dpad_down) {
            rDelivery.setPosition(.7);
        }
        //Open
        if (theOpMode.gamepad1.left_bumper) {
            rCollection.setPosition(.71);
            lCollection.setPosition(.31);
        }
        //Close
        else if (theOpMode.gamepad1.right_bumper) {
            rCollection.setPosition(.61);
            lCollection.setPosition(.46);
        }
        /*
        if (theOpMode.gamepad1.x || theOpMode.gamepad2.x) {
            collection.setPower(.5);
        }
        else if (theOpMode.gamepad1.y || theOpMode.gamepad2.y) {
            collection.setPower(-.75);
        }
        else {
            collection.setPower(0);
        }

         */
        if (theOpMode.gamepad1.dpad_left || theOpMode.gamepad2.dpad_left) {
            box.setPosition(.1);
        }
        else if (theOpMode.gamepad1.dpad_right || theOpMode.gamepad2.dpad_right) {
            box.setPosition(.85);
        }
        if (theOpMode.gamepad2.a) {
            hang.setPower(.6);
        }
        else if (theOpMode.gamepad2.b) {
            hang.setPower(-.6);
        }
        else {
            hang.setPower(0);
        }

        if (theOpMode.gamepad2.left_bumper) {
            hangS.setPower(.9);
        }
        else if (theOpMode.gamepad2.right_bumper) {
            hangS.setPower(-.9);
        }
        else {
            hangS.setPower(0);
        }
        /*
        if (cBeam.getState() == false) {
            theOpMode.telemetry.addData("Beam", "Broken");
        } else {
            theOpMode.telemetry.addData("Beam", "NOT Broken");
        }

         */
    }

    public void openClaw() {
        rCollection.setPosition(.71);
        lCollection.setPosition(.31);
    }
    public void closeClaw() {
        rCollection.setPosition(.61);
        lCollection.setPosition(.46);
    }
    public void deliver() {
        rDelivery.setPosition(.7);
    }
    public void collect() {
        rDelivery.setPosition(.4);

    }
}
