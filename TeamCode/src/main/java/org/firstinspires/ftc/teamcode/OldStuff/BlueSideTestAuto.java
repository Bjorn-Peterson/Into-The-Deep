package org.firstinspires.ftc.teamcode.OldStuff;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
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
import org.opencv.core.Mat;


@Config
@Autonomous(name = "BLUE_TEST_AUTO_PIXEL", group = "Autonomous")
public class BlueSideTestAuto extends LinearOpMode {


    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        PIDF pidf = new PIDF(hardwareMap, this);
        Lift lift = new Lift(hardwareMap, this, 145.1, 1, 1.15);


        Action toDeliver = drive.actionBuilder(initialPose).
                splineToLinearHeading(new Pose2d(12,25,Math.toRadians(-25)), Math.toRadians(0)).
                build();
        Action score2 = drive.actionBuilder(initialPose).
                splineToLinearHeading(new Pose2d(7, 28, Math.toRadians(-2)), Math.toRadians(0)).
                build();
        Action score3 = drive.actionBuilder(initialPose).
                turnTo(30, new TurnConstraints(30, -30, 30)).build();
        Action collect3 = drive.actionBuilder(initialPose).
                strafeToLinearHeading(new Vector2d(10, 19), Math.toRadians(36)).
                build();
        Action score4 = drive.actionBuilder(initialPose).
                splineToLinearHeading(new Pose2d(8, 20, Math.toRadians(-37)), Math.toRadians(0)).
                build();
        Action sub = drive.actionBuilder(initialPose).
                splineTo(new Vector2d(50, -10), Math.toRadians(-90)).
                build();
        Action afterSub = drive.actionBuilder(initialPose).
                setTangent(-30).
                setReversed(true).
                splineTo(new Vector2d(11, 20), Math.toRadians(145)).
                build();
        Action collect6 = drive.actionBuilder(initialPose).
                setReversed(false).
                splineTo(new Vector2d(52, -13), Math.toRadians(-90)).
                build();
        Action end = drive.actionBuilder(initialPose).
                setReversed(false).
                turnTo(Math.toRadians(90), new TurnConstraints(30, -30, 30))
                .build();
        Action jk = drive.actionBuilder(initialPose).
                setTangent(Math.toRadians(90)).
                strafeTo(new Vector2d(12, 15)).
                turnTo(Math.toRadians(-90), new TurnConstraints(30, -30, 30)).
                build();

        Actions.runBlocking(pidf.initPositions());

            waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(toDeliver, lift.liftAction(), pidf.extendCollection()),
                        pidf.collectRun(),
                        lift.liftAction(),
                        new ParallelAction(score2, pidf.collectRun()),
                        new ParallelAction(score3, lift.liftAction()),
                        new ParallelAction(collect3, pidf.collectRun()),
                        new ParallelAction(score4, lift.liftAction()),
                        new ParallelAction(sub, pidf.retractCollection()),
                        pidf.collectRun(),
                        new ParallelAction(afterSub, pidf.retractCollection()),
                        lift.liftAction(),
                        new ParallelAction(collect6, pidf.retractCollection()),
                        pidf.collectRun(),
                        new ParallelAction(end, pidf.retractCollection())
                        ,new ParallelAction(jk, lift.liftAction())));
    }
}