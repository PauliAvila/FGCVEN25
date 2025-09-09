package org.firstinspires.ftc.teamcode.TeleOp.controllers;

import static org.firstinspires.ftc.teamcode.TeleOp.controllers.HangingController.hangingCore;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.TeleOp.RobotMap;

@Config
public class AcceleratorController {
    public enum acceleratorStatus {
        ACCELERATE,
        OFF,

    }

    public static acceleratorStatus currentStatus = acceleratorStatus.ACCELERATE;
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



            }
        }
    }
}