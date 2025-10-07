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
        FREE
    }

    public static liftStatus currentStatus = liftStatus.INIT;
    public liftStatus previousStatus = null;

    public static DcMotorEx Extend = null;

    public int init_position = -50;
    public static int collect_position = 2400;

    public static int rightLiftSpeed = 2500;


    public int currentPosition = init_position;

    public ExtendController(RobotMap robot) {
        Extend = robot.Extend;
    }

    public void update(float target) {

        if (currentStatus != previousStatus) {
            previousStatus = currentStatus;

            switch (currentStatus) {

                case INIT:
                    Extend.setTargetPosition(init_position);
                    Extend.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    Extend.setVelocity(rightLiftSpeed);

                    currentPosition = init_position;
                    break;

                case POWEROFF:
                    Extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    Extend.setPower(0);
                    break;

                case  COLLECT:
                    Extend.setTargetPosition(collect_position);
                    Extend.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    Extend.setVelocity(rightLiftSpeed);

                    currentPosition = collect_position;
                    break;

                case FREE:
                    Extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    break;
            }
        }
    }
    }
