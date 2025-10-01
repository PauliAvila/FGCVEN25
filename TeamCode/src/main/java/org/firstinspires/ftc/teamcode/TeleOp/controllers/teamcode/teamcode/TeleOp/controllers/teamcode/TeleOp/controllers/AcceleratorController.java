package org.firstinspires.ftc.teamcode.TeleOp.controllers.teamcode.teamcode.TeleOp.controllers.teamcode.TeleOp.controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.TeleOp.controllers.teamcode.teamcode.TeleOp.controllers.teamcode.TeleOp.RobotMap;

@Config
public class AcceleratorController {
    public enum acceleratorStatus {
        ACCELERATE,
        OFF,
        DESACCELERATE

    }

    public static acceleratorStatus currentStatus = acceleratorStatus.OFF;
    public acceleratorStatus previousStatus = null;
    public static DcMotorEx Accelerator = null ;




    public AcceleratorController(RobotMap robot) {
        Accelerator = robot.Accelerator;
    }

    public void update() {
        if (currentStatus != previousStatus) {
            previousStatus = currentStatus;

            switch (currentStatus) {

                case ACCELERATE:
                    Accelerator.setPower(1);
                    break;

                case OFF:
                    Accelerator.setPower(0);
                    break;

                case DESACCELERATE:
                    Accelerator.setPower(-1);
                    break;



            }
        }
    }
}