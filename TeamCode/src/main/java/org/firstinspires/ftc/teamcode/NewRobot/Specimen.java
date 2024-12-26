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
@Autonomous(name = "Specimen", group = "Autonomous")
public class Specimen extends LinearOpMode {


    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        PIDF pidf = new PIDF(hardwareMap, this);
        Lift lift = new Lift(hardwareMap, this, 145.1, 1, 1.15);


        Action preload = drive.actionBuilder(initialPose).
                strafeTo(new Vector2d(32.3, 0)).
                build();
        Action toCollect = drive.actionBuilder(new Pose2d(32.3, 0, 0)).
                waitSeconds(.6).
                strafeTo(new Vector2d(12.5, -47)).
                build();
        Action collect1 = drive.actionBuilder(new Pose2d(11, -47, 0)).
                strafeTo(new Vector2d(12.5, -56)).
                waitSeconds(.25).
                build();
        Action collect2 = drive.actionBuilder(new Pose2d(13, -45, 0)).
                turn(Math.toRadians(-35)).
                waitSeconds(.25).
                build();
        Action toDeliver = drive.actionBuilder(new Pose2d(13, -45, 0)).
                waitSeconds(.4).
                strafeTo(new Vector2d(1, -30)).
                build();
        Action deliver1 = drive.actionBuilder(new Pose2d(1, -30, 0)).
                strafeTo(new Vector2d(31.5, 8)).
                build();
        Action toDeliver2 = drive.actionBuilder(new Pose2d(31.5, 8, 0)).
                strafeTo(new Vector2d(1, -30)).
                build();


        Actions.runBlocking(pidf.initPositions());
        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(preload, lift.liftMid(), pidf.retractCollection()),
                        new ParallelAction(toCollect, lift.specDeliver(), pidf.retractCollection()),
                        pidf.collectRun(),
                        new ParallelAction(collect1, lift.pickup()),

                        pidf.collectRun(),
                        new ParallelAction(collect2, lift.pickup()),
                        pidf.collectRun(),
                        new ParallelAction(toDeliver, lift.pickup()),
                        new ParallelAction(deliver1, lift.liftMid(), pidf.retractCollection()),
                        new ParallelAction(toDeliver2, lift.specDeliver(), pidf.retractCollection())

                ));
    }
}