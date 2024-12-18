package org.firstinspires.ftc.teamcode.OldStuff;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.NewRobot.AutoActions.CollectionActions;
import org.firstinspires.ftc.teamcode.NewRobot.Lift;
import org.opencv.core.Mat;

import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "BLUE_TEST_AUTO_PIXEL", group = "Autonomous")
public class BlueSideTestAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        CollectionActions collectionActions = new CollectionActions(hardwareMap);
        Lift lift = new Lift(hardwareMap, this, 145.1, 1, 1.15);

        TrajectoryActionBuilder path1 = drive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(0))
                        .lineToX(20)
                                .waitSeconds(2);
        Action toDeliver = drive.actionBuilder(initialPose).
                setTangent(Math.toRadians(0)).
                splineToLinearHeading(new Pose2d(20,20,35), Math.PI / 2).
                build();
        Action collect1 = drive.actionBuilder(initialPose).
                setTangent(Math.toRadians(35)).
                lineToX(30).build();

        // actions that need to happen on init; for instance, a claw tightening.




            waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        toDeliver,
                        lift.liftAction()));

        Actions.runBlocking(
            new ParallelAction(collect1,
                    collectionActions.collect()));
    }
}