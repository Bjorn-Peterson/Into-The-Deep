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
@Autonomous(name = "BlueSpecimen", group = "Autonomous")
public class BlueSpecimen extends LinearOpMode {


    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        PIDF pidf = new PIDF(hardwareMap, this);
        Lift lift = new Lift(hardwareMap, this, 145.1, 1, 1.15);


        Action preload = drive.actionBuilder(initialPose).
                afterDisp(6, lift.spec()).
                strafeTo(new Vector2d(32, 0)).
                waitSeconds(.2).
                strafeTo(new Vector2d(28, 0)).
                strafeTo(new Vector2d(13, -47)).
                build();

        Action collect1 = drive.actionBuilder(new Pose2d(11, -47, 0)).
                strafeTo(new Vector2d(13, -56)).
                waitSeconds(.25).
                build();

        Action toDeliver = drive.actionBuilder(new Pose2d(11, -45, 0)).
                waitSeconds(.4).
                strafeTo(new Vector2d(8, -36)).
                strafeTo(new Vector2d(1, -36)).
                build();
        Action deliver1 = drive.actionBuilder(new Pose2d(1, -33, 0)).
                afterDisp(10, lift.spec()).
                strafeTo(new Vector2d(29, 7)).
                strafeTo(new Vector2d(33, 7)).
                build();
        Action toDeliver2 = drive.actionBuilder(new Pose2d(32.5, 8, 0)).
                waitSeconds(.3).
                strafeTo(new Vector2d(8, -37)).
                afterDisp(0, lift.pickup()).
                strafeTo(new Vector2d(1, -37)).
                build();
        Action deliver2 = drive.actionBuilder(new Pose2d(1, -30, 0)).
                afterDisp(10, lift.spec()).
                strafeTo(new Vector2d(29, 10)).
                strafeTo(new Vector2d(33, 10)).
                build();
        Action toDeliver3 = drive.actionBuilder(new Pose2d(32.5, 10, 0)).
                waitSeconds(.28).
                strafeTo(new Vector2d(8, -37)).
                afterDisp(0, lift.pickup()).
                strafeTo(new Vector2d(1, -37)).
                build();
        Action deliver3 = drive.actionBuilder(new Pose2d(1, -30, 0)).
                afterDisp(10, lift.spec()).
                strafeTo(new Vector2d(28, 13)).
                strafeTo(new Vector2d(33, 13)).
                build();
        Action toDeliver4 = drive.actionBuilder(new Pose2d(33, 10, 0)).
                waitSeconds(.28).
                strafeTo(new Vector2d(8, -37)).
                afterDisp(0, lift.pickup()).
                strafeTo(new Vector2d(1, -37)).
                build();



        Actions.runBlocking(pidf.initPositions());
        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(preload, pidf.retractCollection()),
                        pidf.collectRun(),
                        new ParallelAction(collect1, lift.pickup()),
                        pidf.collectRun(),
                        new ParallelAction(toDeliver, lift.pickup()),
                        new ParallelAction(deliver1, lift.liftMid(), pidf.retractCollection()),
                        new ParallelAction(toDeliver2, lift.specDeliver(), pidf.retractCollection()),

                        new ParallelAction(deliver2, pidf.retractCollection()),
                        new ParallelAction(toDeliver3, pidf.retractCollection()),
                        new ParallelAction(deliver3,pidf.retractCollection()),
                        new ParallelAction(toDeliver4, pidf.retractCollection())


                ));
    }
}