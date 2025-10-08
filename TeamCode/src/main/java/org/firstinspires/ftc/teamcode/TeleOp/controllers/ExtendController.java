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

    public static DcMotorEx extend = null;

    public int init_position = -200;
    public static int collect_position = 2400;

    public static int rightLiftSpeed = 2500;


    public int currentPosition = init_position;

    public ExtendController(RobotMap robot) {
        extend = robot.extend;
    }

    public void update(float target) {

        if (currentStatus != previousStatus) {
            previousStatus = currentStatus;

            switch (currentStatus) {

                case INIT:
                    extend.setTargetPosition(init_position);
                    extend.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    extend.setVelocity(rightLiftSpeed);

                    currentPosition = init_position;
                    break;

                case POWEROFF:
                    extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    extend.setPower(0);
                    break;

                case  COLLECT:
                    extend.setTargetPosition(collect_position);
                    extend.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    extend.setVelocity(rightLiftSpeed);
                    currentPosition = collect_position;
                    break;

            }
        }
    }
    }
