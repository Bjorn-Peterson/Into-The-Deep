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
import org.firstinspires.ftc.teamcode.NewRobot.Lift;
import org.firstinspires.ftc.teamcode.OldStuff.PIDF;


@Config
@Autonomous(name = "SampleTest", group = "Autonomous")
public class SampleTest extends LinearOpMode {


    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        PIDF pidf = new PIDF(hardwareMap, this);
        Lift lift = new Lift(hardwareMap, this, 145.1, 1, 1.15);


        Action toDeliver = drive.actionBuilder(initialPose).
                splineToLinearHeading(new Pose2d(11.5,25,Math.toRadians(-27)), Math.toRadians(0)).
                build();
        Action score2 = drive.actionBuilder(new Pose2d(11, 23, Math.toRadians(-27))).
                turn(Math.toRadians(25)).
                build();
        Action score3 = drive.actionBuilder(initialPose).
                turnTo(30, new TurnConstraints(30, -30, 30)).build();
        Action collect3 = drive.actionBuilder(initialPose).
                strafeToLinearHeading(new Vector2d(10, 19), Math.toRadians(36)).
                build();
        Action score4 = drive.actionBuilder(new Pose2d(10, 19, -45)).
                strafeTo(new Vector2d(4, 17)).
                build();
        Action sub = drive.actionBuilder(new Pose2d(10, 19, 0)).
                afterDisp(40, pidf.subCollect()).
                splineTo(new Vector2d(49, -10), Math.toRadians(-90)).
                build();
        Action afterSub = drive.actionBuilder(new Pose2d(50,-12,-45)).
                afterDisp(20, lift.liftUp()).
                strafeToConstantHeading(new Vector2d(4, 16)).
                build();
        Action collect6 = drive.actionBuilder(new Pose2d(10, 19, 0)).
                setReversed(false).
                afterDisp(40, pidf.subCollect()).
                splineTo(new Vector2d(48, -13), Math.toRadians(-90)).
                build();
        Action jk = drive.actionBuilder(new Pose2d(52,-13,-45)).
                afterDisp(20, lift.liftUp()).
                strafeToConstantHeading(new Vector2d(4, 16)).
                build();
        Action num7 = drive.actionBuilder(new Pose2d(5, 17, -45)).
                strafeToLinearHeading(new Vector2d(2, -13), -90).
                build();
        Action score7 = drive.actionBuilder(new Pose2d(5,-20,-45)).
                strafeToConstantHeading(new Vector2d(6, 16)).
                build();
        Action heading = drive.actionBuilder(new Pose2d(30, 19, -80)).
                setTangent(-80).
                strafeTo(new Vector2d(20, -100)).
                build();
        Action collect8 = drive.actionBuilder(new Pose2d(10, 19, 0)).
                setReversed(false).
                afterDisp(48, pidf.subCollect()).
                splineTo(new Vector2d(52, -14), Math.toRadians(-90)).
                build();
        Action score8 = drive.actionBuilder(new Pose2d(50, -13, -45)).
                afterDisp(20, lift.liftUp()).
                strafeToConstantHeading(new Vector2d(8, 15.5)).
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
                        new ParallelAction(sub, lift.liftDown()),
                        new ParallelAction(afterSub, pidf.retractCollection()),
                        new ParallelAction(collect6, lift.liftDown()),
                        new ParallelAction(jk, pidf.retractCollection()),
                        new ParallelAction(collect8, lift.liftDown()),
                        new ParallelAction(score8, pidf.retractCollection()),
                        new ParallelAction(num7, pidf.collectRun(), lift.liftDown()),
                        new ParallelAction(score7, pidf.retractCollection(), lift.liftUp()),
                        new ParallelAction(heading, pidf.megaExtend(), lift.liftDown())

                ));
    }
}