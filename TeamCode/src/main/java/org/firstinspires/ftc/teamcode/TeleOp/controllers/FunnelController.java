package org.firstinspires.ftc.teamcode.TeleOp.controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.TeleOp.RobotMap;

@Config
public class  FunnelController{

    public enum FunnelStatus {
        INIT,
        HIGH,

    }

    public static FunnelStatus currentStatus = FunnelStatus.INIT;
    public FunnelStatus previousStatus = null;

    public Servo leftFunnel = null;
    public Servo rightFunnel = null;

    public static double init_position = 0.15;
    public static double high_position = 0.9;




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
