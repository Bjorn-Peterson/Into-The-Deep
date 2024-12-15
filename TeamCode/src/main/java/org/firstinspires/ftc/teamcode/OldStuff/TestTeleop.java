package org.firstinspires.ftc.teamcode.OldStuff;
//Sensor

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TestTeleop extends OpMode {
    ri3d ri3d;

    private PIDController controller;
    public static double p = 0.01, i = 0, d = 0.0001;
    public static double f = 0;
    public static int target;
    private final double ticksPerInch = (145.1) / (1.15 * 3.14);

    public DcMotorEx lift;
    public DcMotorEx rLift;

    public Servo claw;
    public Servo deliveryS;

    double closed = .85;
    double open = .7;
    double frontSpec = .05;
    double backSpec = .54;
    double transferPos = .122;
    double midPos = .3;
    DigitalChannel cBeam;
    DigitalChannel dBeam;
    DigitalChannel lSwitch;



    @Override
    public void init() {

        controller = new PIDController(p, i, d);
        ri3d = new ri3d(hardwareMap, this, 384.5, 1, 4);

        lift = hardwareMap.get(DcMotorEx.class, "leftMotor");
        rLift = hardwareMap.get(DcMotorEx.class, "rightMotor");
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rLift.setDirection(DcMotorSimple.Direction.REVERSE);


    }

        @Override
        public void loop() {


        target = 400;


        controller.setPID(p, i, d);
        int curPos = lift.getCurrentPosition();
        double pid = controller.calculate(curPos, target);
        double ff = Math.cos(Math.toRadians(target / ticksPerInch)) * f;
        double power = pid + ff;
        lift.setPower(power);
        rLift.setPower(power);
        telemetry.addData("pos", curPos);
        telemetry.addData("target", target);
        telemetry.update();
    }


}

