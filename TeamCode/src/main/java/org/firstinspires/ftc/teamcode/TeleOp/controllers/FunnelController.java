package org.firstinspires.ftc.teamcode.TeleOp.controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.TeleOp.RobotMap;

@Config
public class  FunnelController{

    public enum RampStatus {
        INIT,
        HIGH,

    }

    public static RampStatus currentStatus = RampStatus.INIT;
    public RampStatus previousStatus = null;

    public Servo leftFunnel = null;
    public Servo rightFunnel = null;

    public static double init_position = 0;
    public static double high_position = 1;




    public double currentPosition = init_position;

    public FunnelController (RobotMap robot) {
        this.leftFunnel = robot.leftFunnel;
        this.rightFunnel = robot.rightFunnel;

    }

    public void update() {
        if (currentStatus != previousStatus) {
            previousStatus = currentStatus;

            switch (currentStatus) {
                case INIT: {
                    leftFunnel.setPosition(init_position);
                    rightFunnel.setPosition(1-init_position);
                    currentPosition = init_position;
                    break;
                }

                case HIGH: {
                    leftFunnel.setPosition(high_position);
                    rightFunnel.setPosition(1-high_position);
                    currentPosition = high_position;
                    break;
                }

            }
        }
    }
}
