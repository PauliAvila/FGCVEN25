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

    public Servo leftramp = null;
    public Servo rightramp = null;

    public  static double init_positionR = 1;
    public  static double init_positionL = 1;
    public static double high_positionR = 0.4;
    public static double high_positionL = 0.4;
    public  static double init_position ;
    public  static double high_position ;




    public double currentPosition = init_position;

    public RampController (RobotMap robot) {
        this.rightramp =  robot.rightramp;
        this.leftramp =  robot.leftramp;

    }

    public void update() {
        if (currentStatus != previousStatus) {
            previousStatus = currentStatus;

            switch (currentStatus) {
                case INIT: {
                    rightramp.setPosition(init_positionR);
                    leftramp.setPosition(1-init_positionL);
                    currentPosition = init_position;
                    break;
                }

                case HIGH: {
                    rightramp.setPosition(high_positionR);
                    leftramp.setPosition(1-high_positionL);
                    currentPosition = high_position;
                    break;
                }

                }
            }
        }
    }
