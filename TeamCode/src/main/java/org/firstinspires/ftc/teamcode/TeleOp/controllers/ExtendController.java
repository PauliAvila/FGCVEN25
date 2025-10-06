package org.firstinspires.ftc.teamcode.TeleOp.controllers;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

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

    public static DcMotorEx Extend = null;

    public int init_position = 0;
    public static int collect_position = 2140;

    public static int rightLiftSpeed = 2500;
    public static int leftLiftSpeed = 2500;

    public int currentPosition = init_position;

    public ExtendController(RobotMap robot) {
        Extend = robot.Extend;


    }

    public void update(int target) {

        // Si el estado ha cambiado
        double distanciaExtend = ExtendController.Extend.getCurrentPosition();



        if (distanciaExtend < 2390 ) {
            Extend.setPower(gamepad2.left_stick_y);
        } else if (distanciaExtend > 2390) {
            Extend.setPower(0.1);
        } else if (distanciaExtend < -20) {
            Extend.setPower(-0.1);
        } else {
            Extend.setPower(0);
        }

        /*if (currentStatus != previousStatus) {
            previousStatus = currentStatus;

            switch (currentStatus) {

                case INIT:
                    rightExtend.setTargetPosition(init_position);
                    rightExtend.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    rightExtend.setVelocity(rightLiftSpeed);

                    currentPosition = init_position;
                    break;

                case POWEROFF:
                    rightExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightExtend.setPower(0);
                    break;

                case  COLLECT:
                    rightExtend.setTargetPosition(collect_position);
                    rightExtend.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    rightExtend.setVelocity(rightLiftSpeed);

                    currentPosition = collect_position;
                    break;
            }*/
        }
    }
