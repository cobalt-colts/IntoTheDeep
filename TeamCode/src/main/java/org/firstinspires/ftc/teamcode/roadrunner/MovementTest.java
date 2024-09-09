package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name="Movement Test Auton", group="Testing")
public class MovementTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-12, -63, Math.toRadians(90)));

        Action placePurplePixel = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(0, -24), Math.PI / 2)
                .
                .build();

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        placePurplePixel
                )
        );
    }
}
