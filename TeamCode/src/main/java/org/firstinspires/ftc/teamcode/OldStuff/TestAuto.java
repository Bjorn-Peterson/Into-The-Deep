package org.firstinspires.ftc.teamcode.OldStuff;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.OldStuff.PIDF;
import org.firstinspires.ftc.teamcode.NewRobot.Lift;


@Config
@Autonomous(name = "TestAuto", group = "Autonomous")
public class TestAuto extends LinearOpMode {


    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        PIDF pidf = new PIDF(hardwareMap, this);
        Lift lift = new Lift(hardwareMap, this, 145.1, 1, 1.15);


        Action toDeliver = drive.actionBuilder(initialPose).
                splineToLinearHeading(new Pose2d(12,24,Math.toRadians(-25)), Math.toRadians(0)).
                build();
        Action score2 = drive.actionBuilder(new Pose2d(11.5, 23, Math.toRadians(-27))).
                turn(Math.toRadians(25)).
                build();
        Action score3 = drive.actionBuilder(initialPose).
                turnTo(30, new TurnConstraints(30, -30, 30)).build();
        Action collect3 = drive.actionBuilder(initialPose).
                strafeToLinearHeading(new Vector2d(12, 19), Math.toRadians(36)).
                build();
        Action score4 = drive.actionBuilder(new Pose2d(10, 19, -45)).
                strafeTo(new Vector2d(3, 15)).
                build();
        Action sub = drive.actionBuilder(new Pose2d(10, 19, 0)).
                afterDisp(6, lift.liftDown()).
                afterDisp(50, pidf.subCollect()).
                splineTo(new Vector2d(51, -10), Math.toRadians(-90)).
                build();
        Action afterSub = drive.actionBuilder(new Pose2d(50,-12,Math.toRadians(-90))).
                setReversed(true).
                afterDisp(27, lift.liftUp()).
                splineTo(new Vector2d(10, 15), Math.toRadians(20)).
                build();
        Action collect6 = drive.actionBuilder(new Pose2d(12, 16, 0)).
                setReversed(false).
                afterDisp(8, lift.liftDown()).
                afterDisp(50, pidf.subCollect()).
                splineTo(new Vector2d(57, -13), Math.toRadians(-90)).
                build();
        Action jk = drive.actionBuilder(new Pose2d(52,-13,Math.toRadians(-90))).
                setReversed(true).
                afterDisp(30, lift.liftUp()).
                splineTo(new Vector2d(10, 15), Math.toRadians(20)).
                build();
        Action num7 = drive.actionBuilder(new Pose2d(5, 15, -45)).
                strafeToLinearHeading(new Vector2d(2, -13), -90).
                build();
        Action score7 = drive.actionBuilder(new Pose2d(9,-8,-45)).
                afterDisp(5, lift.liftUp()).
                strafeToConstantHeading(new Vector2d(9, 18)).
                build();
        Action heading = drive.actionBuilder(new Pose2d(10, 19, 0)).
                setReversed(false).
                afterDisp(8, lift.liftDown()).
                afterDisp(50, pidf.subCollect()).
                splineTo(new Vector2d(54, -14), Math.toRadians(-90)).
                build();
        Action collect8 = drive.actionBuilder(new Pose2d(10, 16, 0)).
                setReversed(false).
                afterDisp(8, lift.liftDown()).
                afterDisp(50, pidf.subCollect()).
                splineTo(new Vector2d(52, -14), Math.toRadians(-90)).
                build();
        Action score8 = drive.actionBuilder(new Pose2d(50, -13, -45)).
                afterDisp(33, lift.liftUp()).
                strafeToConstantHeading(new Vector2d(6.5, 14)).
                build();

        Actions.runBlocking(pidf.initPositions());

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(toDeliver, lift.liftUp(), pidf.extendCollection()),
                        new ParallelAction(lift.liftDown(), pidf.collectRun()),
                        lift.liftUp(),
                        new ParallelAction(score2, lift.liftDown(), pidf.extendCollection()),
                        pidf.collectRun(),
                        new ParallelAction(score3, lift.liftUp()),
                        new ParallelAction(collect3, pidf.collectRun(),lift.liftDown()),
                        new ParallelAction(score4, lift.liftUp(), pidf.retractCollection()),
                        new ParallelAction(num7, pidf.collectRun(), lift.liftDown()),
                        new ParallelAction(score7, pidf.retractCollection()),
                        new ParallelAction(sub),
                        new ParallelAction(afterSub, pidf.retractCollection()),
                        new ParallelAction(collect6),
                        new ParallelAction(jk, pidf.retractCollection()),
                        new ParallelAction(collect8),
                        new ParallelAction(score8, pidf.retractCollection()),
                        new ParallelAction(heading)

                ));
    }
}






