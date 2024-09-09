package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public class RedAuton extends LinearOpMode {
    public DcMotor leftWrist, rightWrist, slide;
    public Servo leftClaw, rightClaw, leftClawWrist, rightClawWrist;
    public MecanumDrive drive;
    public void runOpMode(){
            drive = new MecanumDrive(hardwareMap, new Pose2d(12,-60,Math.toRadians(90)));

    }

}
