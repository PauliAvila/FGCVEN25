package org.firstinspires.ftc.teamcode.TeleOp.controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.TeleOp.RobotMap;

@Config
public class HangingController {
    public enum hangingStatus {
        HANG,
        POWEROFF,
        UNHANG,

    }

    public static hangingStatus currentStatus = hangingStatus.POWEROFF;
    public hangingStatus previousStatus = null;
    public static DcMotorEx hanging = null ;
    public static DcMotorEx hangingCore = null;
    public static int coreSpeed = 1000;
    public static int ultraSpeed = 1000;


    public HangingController(RobotMap robot) {
        hanging = robot.hanging;
        hangingCore = robot.hangingCore;
    }

    public void update(int target) {
        if (currentStatus != previousStatus) {
            previousStatus = currentStatus;

            switch (currentStatus) {

                case HANG:
                    hanging.setPower(1);

                    hangingCore.setPower(1);

                    break;

                case POWEROFF:
                    hanging.setVelocity(0);
                    hangingCore.setVelocity(0);
                    break;

                case  UNHANG:
                    hanging.setPower(-1);
                    hangingCore.setPower(-1);

                    break;

            }
        }
    }
}