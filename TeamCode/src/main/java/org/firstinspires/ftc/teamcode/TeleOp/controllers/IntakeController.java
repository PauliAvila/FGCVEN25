package org.firstinspires.ftc.teamcode.TeleOp.controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.TeleOp.RobotMap;

@Config
public class IntakeController {
    public enum intakeStatus {
        FORWARD,
        POWEROFF,
        REVERSE,

    }

    public static intakeStatus currentStatus = intakeStatus.FORWARD;
    public intakeStatus previousStatus = null;
    public static DcMotorEx intake = null;

    public IntakeController(RobotMap robot) {
        intake = robot.intake   ;
    }

    public void update(int target) {
        // Si el estado ha cambiado
        if (currentStatus != previousStatus) {
            previousStatus = currentStatus;

            switch (currentStatus) {

                case FORWARD:
                    intake.setPower(0.85);
                    break;

                case POWEROFF:
                    intake.setPower(0);
                    break;

                case  REVERSE:
                    intake.setPower(-0.85);
                    break;

            }
        }
    }
}