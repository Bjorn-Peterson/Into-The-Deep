package org.firstinspires.ftc.teamcode.NewRobot;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.OldStuff.PIDF;


@Config
@Autonomous(name = "RedSpecimen", group = "Autonomous")
public class RedSpecimen extends LinearOpMode {


    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        PIDF pidf = new PIDF(hardwareMap, this);
        Lift lift = new Lift(hardwareMap, this, 145.1, 1, 1.15);


        Action preload = drive.actionBuilder(initialPose).
                afterDisp(5, lift.liftMid()).
                strafeToLinearHeading(new Vector2d(0, 30.3), Math.toRadians(-90)).
                build();
        Action toCollect = drive.actionBuilder(new Pose2d(0, 30, Math.toRadians(-90))).
                afterDisp(14, pidf.specCollect()).
                strafeToLinearHeading(new Vector2d(20, 22), Math.toRadians(40)).
                build();
        Action collect1 = drive.actionBuilder(new Pose2d(20, 22, Math.toRadians(40))).
                strafeToLinearHeading(new Vector2d(24, 20), Math.toRadians(-35)).
                build();
        Action collect2 = drive.actionBuilder(new Pose2d(24, 20, Math.toRadians(-35))).
                strafeToLinearHeading(new Vector2d(29, 21), Math.toRadians(33)).
                build();
        Action spit2 = drive.actionBuilder(new Pose2d(23, 20, Math.toRadians(30))).
                strafeToLinearHeading(new Vector2d(24, 20), Math.toRadians(-35)).
                build();
        Action collect3 = drive.actionBuilder(new Pose2d(24, 20, Math.toRadians(-35))).
                strafeToLinearHeading(new Vector2d(36, 22), Math.toRadians(35)).
                build();
        Action toPickup = drive.actionBuilder(new Pose2d(36, 22, Math.toRadians(35))).
                strafeToLinearHeading(new Vector2d(40, 5), Math.toRadians(90)).
                build();

        Action toPickup2 = drive.actionBuilder(new Pose2d(2, 30.3, Math.toRadians(-90))).
                afterDisp(5, lift.liftDown()).
                strafeToLinearHeading(new Vector2d(40, 7), Math.toRadians(90)).
                build();
        Action pickup = drive.actionBuilder(new Pose2d(40, 5, Math.toRadians(90))).
                strafeToLinearHeading(new Vector2d(40, -2), Math.toRadians(90)).
                build();
        Action deliver2 = drive.actionBuilder(new Pose2d(40, 0, Math.toRadians(90))).
                //afterDisp(10, lift.liftMid()).
                strafeToLinearHeading(new Vector2d(0, 30.3), Math.toRadians(-90)).
                build();
        Action deliver3 = drive.actionBuilder(new Pose2d(40, 0, Math.toRadians(90))).
               // afterDisp(10, lift.liftMid()).
                strafeToLinearHeading(new Vector2d(-2, 30.3), Math.toRadians(-90)).
                build();
        Action deliver4 = drive.actionBuilder(new Pose2d(40, 0, 90)).
               // afterDisp(10, lift.liftMid()).
                strafeToLinearHeading(new Vector2d(-4, 30.3), Math.toRadians(-90)).
                build();



        Actions.runBlocking(pidf.initPositions());
        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(preload, pidf.retractCollection()),
                        lift.specDeliver(),
                        toCollect,
                        new ParallelAction(collect1, lift.liftDown()),
                        pidf.spitOut(),
                        new ParallelAction(collect2, pidf.extendCollection()),
                        pidf.specCollect(),
                        spit2,
                        pidf.spitOut(),
                        new ParallelAction(collect3, pidf.extendCollection()),
                        pidf.collectRun(),
                        //Going to collect 1st from wall
                        new ParallelAction(toPickup, lift.pickup(), pidf.retractCollection()),

                        pickup,
                        lift.liftPickup(),
                        new ParallelAction(deliver2, lift.liftMid()),
                        lift.specDeliver(),
                        //Going to collect 2nd from wall
                        new ParallelAction(toPickup2, pidf.retractCollection()),

                        pickup,
                        lift.liftPickup(),
                        new ParallelAction(deliver3, lift.liftMid()),
                        lift.specDeliver(),
                        new ParallelAction(toPickup2, pidf.retractCollection(), lift.liftDown()),

                        pickup,
                        lift.liftPickup(),
                        new ParallelAction(deliver4, lift.liftMid()),
                        lift.specDeliver(),
                        new ParallelAction(toPickup2, pidf.retractCollection(), lift.liftDown())

//
//3UHJX3

                ));
    }
}