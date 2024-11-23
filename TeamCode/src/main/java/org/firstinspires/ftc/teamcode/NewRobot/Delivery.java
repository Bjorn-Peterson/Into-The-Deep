package org.firstinspires.ftc.teamcode.NewRobot;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.DigitalChannel;

public class Delivery {
    private ElapsedTime runtime = new ElapsedTime();
    public Servo rCollection;
    public Servo lCollection;
    public Servo claw;
    public Servo deliveryS;

    DcMotor collection;
    DcMotor hang;
    DigitalChannel cBeam;


    private OpMode theOpMode;

    public Delivery(HardwareMap hardwareMap, OpMode opMode) {
        theOpMode = opMode;
        rCollection = hardwareMap.get(Servo.class, "rCollection");
        lCollection = hardwareMap.get(Servo.class, "lCollection");
        deliveryS = hardwareMap.get(Servo.class, "delivery");
        claw = hardwareMap.get(Servo.class, "claw");
        //collection = hardwareMap.get(DcMotorEx.class, "collection");


        //   collection.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


//        cBeam = hardwareMap.get(DigitalChannel.class, "beam");
//
//        cBeam.setMode(DigitalChannel.Mode.INPUT);

    }

    public void teleopControls() {
        if (theOpMode.gamepad1.dpad_up) {
            deliveryS.setPosition(.95);
        }
        else if (theOpMode.gamepad1.dpad_down) {
            deliveryS.setPosition(.55);
        }

        if (theOpMode.gamepad1.left_bumper) {
            claw.setPosition(.85);
        }
        else if (theOpMode.gamepad1.right_bumper) {
            claw.setPosition(.75);
        }

        if (theOpMode.gamepad1.dpad_left) {
            lCollection.setPosition(.5);
            rCollection.setPosition(.5);
        }
        else if (theOpMode.gamepad1.dpad_right) {
            lCollection.setPosition(.2);
            rCollection.setPosition(.2);
        }
        //Deliver
    }
}