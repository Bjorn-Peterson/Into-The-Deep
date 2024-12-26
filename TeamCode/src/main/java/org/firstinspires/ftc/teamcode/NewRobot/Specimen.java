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

import java.util.Vector;


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
                strafeTo(new Vector2d(30, 0)).
                build();
        Action toCollect = drive.actionBuilder(initialPose).
                strafeTo(new Vector2d(20, -30)).
                build();
        Action collect1 = drive.actionBuilder(initialPose).
                strafeTo(new Vector2d(20, -35)).
                build();


        Actions.runBlocking(pidf.initPositions());
        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(preload, lift.liftMid()),
                        new ParallelAction(toCollect, lift.specDeliver(), pidf.retractCollection()),
                        pidf.collectRun(),
                        new ParallelAction(collect1, lift.pickup())

                ));
    }
}