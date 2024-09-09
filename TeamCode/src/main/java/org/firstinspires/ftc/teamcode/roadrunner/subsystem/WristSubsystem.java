package org.firstinspires.ftc.teamcode.roadrunner.subsystem;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class WristSubsystem {
    private DcMotor leftWrist, rightWrist;
    public PIDController wristPID = new PIDController(0.2,0,0);

    public WristSubsystem(HardwareMap hardwareMap){
        leftWrist = hardwareMap.get(DcMotor.class,"leftWrist");
        rightWrist = hardwareMap.get(DcMotor.class,"rightWrist");
        leftWrist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightWrist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftWrist.setDirection(DcMotor.Direction.REVERSE);
    }

    public class MoveWristAction implements Action {
        int ticks;
        public MoveWristAction(int ticks){
            this.ticks = ticks;
        }
        @Override
        public boolean run(TelemetryPacket packet){

            return false;
        }
    }

    public Action moveWristAction(int ticks){
        return new MoveWristAction(ticks);
    }
}
