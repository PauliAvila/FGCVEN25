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
    public static DcMotorEx hanging = null;

    public HangingController(RobotMap robot) {
        hanging = robot.hanging;
    }

    public void update(int target) {
        if (currentStatus != previousStatus) {
            previousStatus = currentStatus;

            switch (currentStatus) {

                case HANG:
                    hanging.setPower(0.85);
                    break;

                case POWEROFF:
                    hanging.setPower(0);
                    break;

                case  UNHANG:
                    hanging.setPower(-0.85);
                    break;

            }
        }
    }
}