package org.firstinspires.ftc.teamcode.OldStuff;
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
                setTangent(Math.toRadians(35)).
                splineToLinearHeading(new Pose2d(12,25,Math.toRadians(-25)), Math.toRadians(0)).
                build();
        Action score2 = drive.actionBuilder(initialPose).
                turnTo(Math.toRadians(-9)).
                build();
        Action collect3 = drive.actionBuilder(initialPose).
                strafeToLinearHeading(new Vector2d(10, 20), Math.toRadians(25)).
                build();
        Action score4 = drive.actionBuilder(initialPose).
                turnTo(Math.toRadians(-25)).
                build();
        Action sub = drive.actionBuilder(initialPose).
                splineTo(new Vector2d(50, -10), Math.toRadians(-90)).
                build();
        Action afterSub = drive.actionBuilder(initialPose).
                setReversed(true).
                splineToLinearHeading(new Pose2d(-5, 5, Math.toRadians(-10)), Math.toRadians(0)).
                build();

        Actions.runBlocking(pidf.initPositions());

            waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(toDeliver, lift.liftAction()),
                        pidf.collectRun(),
                        lift.liftAction(),
                        new ParallelAction(score2, pidf.collectRun()),
                        lift.liftAction(),
                        new ParallelAction(collect3, pidf.collectRun()),
                        new ParallelAction(score4, lift.liftAction()),
                        sub,
                        pidf.collectRun(),
                        new ParallelAction(afterSub, lift.liftAction())));
    }
}