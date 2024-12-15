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
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class PDFL extends LinearOpMode {
    public enum LiftState {
        START,
        EXTEND,
        EXTENDED,
        RETRACT
    }
    LiftState liftState = LiftState.START;
    ri3d ri3d;

    private PIDController controller;
    public static double p = 0.01, i = 0, d = 0.0001;
    public static double f = 0;
    public static int target;
    private final double ticksPerInch = (145.1) / (1.15 * 3.14);

    public DcMotorEx leftMotor;
    public DcMotorEx rightMotor;

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
    Servo rDelivery;
    ElapsedTime liftTimer = new ElapsedTime();
    int retracted = 20;
    boolean liftDone;
    boolean liftStart = false;



    @Override
        public void runOpMode() {

        controller = new PIDController(p, i, d);
        ri3d = new ri3d(hardwareMap, this, 384.5, 1, 4);

        leftMotor = hardwareMap.get(DcMotorEx.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightMotor");
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);




            waitForStart();

    liftStart = true;
    target = 400;
    //ri3d.encoderDrive(.3, 3, 3);






        switch (liftState) {
            case START:
                if (liftStart) {
                    liftState = LiftState.EXTEND;
                }
                break;
            case EXTEND:
                if (Math.abs(leftMotor.getCurrentPosition() - target) < 100) {
                    rDelivery.setPosition(.8);
                    liftTimer.reset();
                    liftState = LiftState.EXTENDED;
                }
                break;
            case EXTENDED:
                if (liftTimer.seconds() >= 2) {
                    liftState = LiftState.RETRACT;
                    rDelivery.setPosition(.4);
                }
                break;
            case RETRACT:
                target = retracted;
                if (Math.abs(leftMotor.getCurrentPosition() - target) < 10) {
                    liftDone = true;
                    liftState = LiftState.START;
                    liftStart = false;
                }
                break;
            default:
                liftState = LiftState.START;
        }


        controller.setPID(p, i, d);
        int curPos = leftMotor.getCurrentPosition();
        double pid = controller.calculate(curPos, target);
        double ff = Math.cos(Math.toRadians(target / ticksPerInch)) * f;
        double power = pid + ff;
        leftMotor.setPower(power);
        rightMotor.setPower(power);
        telemetry.addData("pos", curPos);
        telemetry.addData("target", target);
        telemetry.addData("power", power);
        telemetry.addData("Lift State", liftState);
        telemetry.update();
    }
}



