package org.firstinspires.ftc.teamcode.TeleOp.controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.TeleOp.RobotMap;

@Config
public class RampController {

    public enum RampStatus {
        INIT,
        HIGH,

    }

    public static RampStatus currentStatus = RampStatus.INIT;
    public RampStatus previousStatus = null;

    public DcMotorEx Ramp = null;

    public int init_position = 0;
    public static int high_position = 50;
    public static int rampspeed = 2000;




    public double currentPosition = init_position;

    public RampController (RobotMap robot) {
        this.Ramp = robot.Ramp;

    }

    public void update() {
        if (currentStatus != previousStatus) {
            previousStatus = currentStatus;

            switch (currentStatus) {
                case INIT: {
                    Ramp.setTargetPosition(init_position);
                    Ramp.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    Ramp.setVelocity(rampspeed);

                    currentPosition = init_position;
                    break;
                }

                case HIGH: {
                    Ramp.setTargetPosition(high_position);
                    Ramp.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    Ramp.setVelocity(rampspeed);
                    currentPosition = high_position;
                    break;
                }

                }
            }
        }
    }
