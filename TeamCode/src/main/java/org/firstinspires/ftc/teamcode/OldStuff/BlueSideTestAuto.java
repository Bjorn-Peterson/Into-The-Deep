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
                splineToLinearHeading(new Pose2d(11,25,Math.toRadians(-23)), Math.toRadians(0)).
                build();
        Action score2 = drive.actionBuilder(initialPose).
                strafeToLinearHeading(new Vector2d(10, 25), Math.toRadians(-5)).
                build();

        Actions.runBlocking(pidf.initPositions());

            waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(toDeliver, lift.liftAction()),
                        pidf.collectRun(),
                        new ParallelAction(score2, lift.liftAction())));
    }
}