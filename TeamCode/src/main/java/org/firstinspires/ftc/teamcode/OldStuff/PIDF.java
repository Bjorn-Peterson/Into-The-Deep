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
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDF {
    private PIDController controller;
    public static double p = 0.004, i = 0, d = 0.0001;
    public static double f = 0.12;
    public static int target;
    private final double ticks_in_degree = 100;
    private DcMotorEx collection;


    private ElapsedTime runtime = new ElapsedTime();
    private OpMode theOpMode;

    public PIDF(HardwareMap hardwareMap, OpMode opMode) {
        theOpMode = opMode;
        controller = new PIDController(p, i, d);
        collection = hardwareMap.get(DcMotorEx.class, "collection");
        collection.setDirection(DcMotorSimple.Direction.REVERSE);
        collection.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        collection.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void tele() {
        if (theOpMode.gamepad1.x) {
            target = -190;
        } else if (theOpMode.gamepad1.y) {
            target = -35;
        } else if (theOpMode.gamepad1.a) {
            target = -260;
        }
        controller.setPID(p, i, d);
        int armPos = collection.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
        double power = pid + ff;
        collection.setPower(power);
        theOpMode.telemetry.addData("pos", armPos);
        theOpMode.telemetry.addData("target", target);
        theOpMode.telemetry.update();
    }

    public void auto(double target, double timeoutS) {
        runtime.reset();
        controller.setPID(p, i, d);
        int armPos = collection.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
        double power = pid + ff;
        collection.setPower(power);
        while (((LinearOpMode) theOpMode).opModeIsActive() && armPos > target &&  runtime.seconds() < timeoutS) {
            theOpMode.telemetry.addData("pos", armPos);
            theOpMode.telemetry.addData("target", target);
            theOpMode.telemetry.update();
        }
        collection.setPower(0);
    }
}
