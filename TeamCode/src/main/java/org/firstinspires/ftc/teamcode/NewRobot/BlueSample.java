package org.firstinspires.ftc.teamcode.NewRobot;
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


@Config
@Autonomous(name = "Blue 8", group = "Autonomous")
public class BlueSample extends LinearOpMode {


    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        PIDF pidf = new PIDF(hardwareMap, this);
        Lift lift = new Lift(hardwareMap, this, 145.1, 1, 1.15);


        Action toDeliver = drive.actionBuilder(initialPose).
                afterDisp(1, lift.liftUp()).
                splineToLinearHeading(new Pose2d(13,25,Math.toRadians(-24)), Math.toRadians(0)).
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
                afterDisp(55, pidf.sweeperOut()).
                afterDisp(49, pidf.subCollectBlue()).
                splineTo(new Vector2d(51, -10), Math.toRadians(-90)).
                build();
        Action afterSub = drive.actionBuilder(new Pose2d(52,-11,-45)).
                afterDisp(32, lift.liftUp()).
                strafeToConstantHeading(new Vector2d(7.5, 14)).
                build();
        Action collect6 = drive.actionBuilder(new Pose2d(12, 18, Math.toRadians(-5))).
                setReversed(false).
                afterDisp(8, lift.liftDown()).
                afterDisp(55, pidf.sweeperOut()).
                afterDisp(44, pidf.subCollectBlue()).
                splineTo(new Vector2d(58, -13), Math.toRadians(-90)).
                build();
        Action jk = drive.actionBuilder(new Pose2d(52,-13,-45)).
                afterDisp(35, lift.liftUp()).
                strafeToConstantHeading(new Vector2d(7.5, 14)).
                build();
        Action num7 = drive.actionBuilder(new Pose2d(5, 15, -45)).
                strafeToLinearHeading(new Vector2d(2, -13), -90).
                build();
        Action score7 = drive.actionBuilder(new Pose2d(9,-8,-45)).
                afterDisp(2, lift.liftUp()).
                strafeToConstantHeading(new Vector2d(9, 16)).
                build();
        Action heading = drive.actionBuilder(new Pose2d(10, 19, 0)).
                setReversed(false).
                afterDisp(8, lift.liftDown()).
                afterDisp(55, pidf.sweeperOut()).
                afterDisp(47, pidf.subCollectBlue()).
                splineTo(new Vector2d(54, -14), Math.toRadians(-90)).
                build();
        Action collect8 = drive.actionBuilder(new Pose2d(10, 18, Math.toRadians(-5))).
                setReversed(false).
                afterDisp(8, lift.liftDown()).
                afterDisp(55, pidf.sweeperOut()).
                afterDisp(44, pidf.subCollectBlue()).
                splineTo(new Vector2d(52, -14), Math.toRadians(-90)).
                build();
        Action score8 = drive.actionBuilder(new Pose2d(50, -13, -45)).
                afterDisp(36, lift.liftUp()).
                strafeToConstantHeading(new Vector2d(7.5, 14)).
                build();
        Action score9 = drive.actionBuilder(new Pose2d(50, -13, -45)).
                afterDisp(36, lift.liftUp()).
                strafeToConstantHeading(new Vector2d(7.5, 14)).
                build();

        Actions.runBlocking(pidf.initPositions());

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(toDeliver, pidf.extendCollection()),
                        new ParallelAction(lift.liftDown(), pidf.collectRun()),
                        lift.liftUp(),
                        new ParallelAction(score2, lift.liftDown(), pidf.extendCollection()),
                        pidf.collectRun(),
                        new ParallelAction(score3, lift.liftUp()),
                        new ParallelAction(collect3, pidf.collectRun(),lift.liftDown()),
                        new ParallelAction(score4, lift.liftUp(), pidf.retractCollection()),
                        new ParallelAction(num7, pidf.collectRun(), lift.liftDown()),
                        new ParallelAction(score7, pidf.retractCollection(), pidf.sweeperOut()),
                        new ParallelAction(sub, pidf.sweeperIn()),
                        new ParallelAction(afterSub, pidf.retractCollection()),
                        new ParallelAction(collect6, pidf.sweeperIn()),
                        new ParallelAction(jk, pidf.retractCollection()),
                        new ParallelAction(collect8, pidf.sweeperIn()),
                        new ParallelAction(score8, pidf.retractCollection()),
                        new ParallelAction(heading, pidf.sweeperIn()),
                        new ParallelAction(score9, pidf.retractCollection()),
                        heading

                ));
    }
}






