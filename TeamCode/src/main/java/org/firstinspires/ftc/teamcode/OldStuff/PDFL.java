package org.firstinspires.ftc.teamcode.OldStuff;
//Sensor

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.NewRobot.Delivery;

@Autonomous
public class PDFL extends LinearOpMode {

    private PIDController controller;
    public static double p = 0.01, i = 0, d = 0.0001;
    public static double f = 0;
    public static int target;
    private final double ticksPerInch = 0;

    public DcMotorEx lift;

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
        public void runOpMode() {
        PIDF pidf = new PIDF(hardwareMap, this);
        controller = new PIDController(p, i, d);

        claw = hardwareMap.get(Servo.class, "claw");
        deliveryS = hardwareMap.get(Servo.class, "delivery");
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        cBeam = hardwareMap.get(DigitalChannel.class, "beam");
        cBeam.setMode(DigitalChannel.Mode.INPUT);
        dBeam = hardwareMap.get(DigitalChannel.class, "dBeam");
        dBeam.setMode(DigitalChannel.Mode.INPUT);
        lSwitch = hardwareMap.get(DigitalChannel.class, "lSwitch");
        lSwitch.setMode(DigitalChannel.Mode.INPUT);


            waitForStart();

            pidf.auto(300);
            target = 200;


            controller.setPID(p, i, d);
            int curPos = lift.getCurrentPosition();
            double pid = controller.calculate(curPos, target);
            double ff = Math.cos(Math.toRadians(target / ticksPerInch)) * f;
            double power = pid + ff;
            lift.setPower(power);
            telemetry.addData("pos", curPos);
            telemetry.addData("target", target);
            telemetry.update();
        }


    }

