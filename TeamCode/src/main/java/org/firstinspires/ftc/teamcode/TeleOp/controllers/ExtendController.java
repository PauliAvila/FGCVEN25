package org.firstinspires.ftc.teamcode.TeleOp.controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.TeleOp.RobotMap;

@Config
public class ExtendController {
    public enum liftStatus {
        INIT,
        POWEROFF,
        COLLECT,

    }

    public static liftStatus currentStatus = liftStatus.INIT;
    public liftStatus previousStatus = null;

    public static DcMotorEx rightExtend = null;
    public static DcMotorEx leftExtend = null;

    public int init_position = 0;
    public int collect_position = 2000;
    public static int medium_position = 1150;

    public static int rightLiftSpeed = 2500;
    public static int leftLiftSpeed = 2500;

    public int currentPosition = init_position;

    public ExtendController(RobotMap robot) {
        rightExtend = robot.rightExtend;
        leftExtend = robot.leftExtend;
    }

    public void update(int target) {
        // Si el estado ha cambiado
        if (currentStatus != previousStatus) {
            previousStatus = currentStatus;

            switch (currentStatus) {

                case INIT:
                    rightExtend.setTargetPosition(init_position);
                    rightExtend.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    rightExtend.setVelocity(rightLiftSpeed);

                    leftExtend.setTargetPosition(init_position);
                    leftExtend.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    leftExtend.setVelocity(leftLiftSpeed);

                    currentPosition = init_position;
                    break;

                case POWEROFF:
                    rightExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightExtend.setPower(0);
                    leftExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    leftExtend.setPower(0);
                    break;

                case  COLLECT:
                    rightExtend.setTargetPosition(collect_position);
                    rightExtend.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    rightExtend.setVelocity(rightLiftSpeed);

                   leftExtend.setTargetPosition(collect_position);
                   leftExtend.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                   leftExtend.setVelocity(leftLiftSpeed);

                    currentPosition = collect_position;
                    break;

            }
        }
    }
}