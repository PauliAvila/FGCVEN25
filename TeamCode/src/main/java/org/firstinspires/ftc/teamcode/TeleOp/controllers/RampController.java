package org.firstinspires.ftc.teamcode.TeleOp.controllers;

import com.acmerobotics.dashboard.config.Config;
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

    public Servo leftRamp = null;
    public Servo rightRamp = null;

    public static double init_position;
    public static double high_position ;

    public static double init_positionR = 0.15;
    public static double init_positionL = 0.15;
    public static double high_positionL = 0.4;
    public static double high_positionR = 0.4 ;


    public double currentPosition = init_position;

    public RampController (RobotMap robot) {
        this.leftRamp = robot.leftRamp;
        this.rightRamp = robot.rightRamp;

    }

    public void update() {
        if (currentStatus != previousStatus) {
            previousStatus = currentStatus;

            switch (currentStatus) {
                case INIT: {
                    leftRamp.setPosition(init_positionL);
                    rightRamp.setPosition(1-init_positionR);
                    currentPosition = init_position;
                    break;
                }

                case HIGH: {
                    leftRamp.setPosition(high_positionL);
                    rightRamp.setPosition(1-high_positionR);
                    currentPosition = high_position;
                    break;
                }

                }
            }
        }
    }
