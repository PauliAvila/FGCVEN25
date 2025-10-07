package org.firstinspires.ftc.teamcode.TeleOp.controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.TeleOp.RobotMap;

@Config
public class HugController {

    public enum hugStatus {
        INIT,
        HUG,
        CLOSED,
        LEFT_FLOW,
        RIGHT_FLOW,
        STRAIGHT,
    }

    public static hugStatus currentStatus = hugStatus.CLOSED;
    public hugStatus previousStatus = null;

    public Servo hugleftservo = null;
    public Servo hugrightservo = null;

    public static double init_positionL = 0.93;
    public static double init_positionR =0.95;
    public static double straight_positionL = 0.375;
    public static double straight_positionR = 0.375;
    public static double closed_positionL =  0.99;
    public static double closed_positionR =  0.99;
    public static double left_flow_positionL =  0.2;
    public static double left_flow_positionR =  0.4;
    public static double right_flow_positionL =  0.2;
    public static double right_flow_positionR =  0.4;
    public static double hug_positionL =  0.13;
    public static double hug_positionR =  0.13;


    public double currentPosition = init_positionL;

    public HugController(RobotMap robot) {
        this.hugleftservo = robot.hugleftservo;
        this.hugrightservo = robot.hugrightservo;

    }

    public void update() {
        if (currentStatus != previousStatus) {
            previousStatus = currentStatus;

            switch (currentStatus) {
                case INIT: {
                    hugleftservo.setPosition(init_positionL);
                    hugrightservo.setPosition(1-init_positionR);
                    currentPosition = init_positionL;
                    break;
                }

                case HUG: {

                    hugleftservo.setPosition(hug_positionL);
                    hugrightservo.setPosition(1-hug_positionR);

                    currentPosition = hug_positionL;
                    break;
                }
                case CLOSED: {

                    hugleftservo.setPosition(closed_positionL);
                    hugrightservo.setPosition(1-closed_positionR);

                    currentPosition = closed_positionL;
                    break;
                }
                case STRAIGHT: {

                    hugleftservo.setPosition(straight_positionL);
                    hugrightservo.setPosition(1-straight_positionR);

                    currentPosition = straight_positionL;
                    break;
                }
                case LEFT_FLOW: {

                    hugleftservo.setPosition(left_flow_positionL);
                    hugrightservo.setPosition(1-left_flow_positionR);

                    currentPosition = left_flow_positionL;
                    break;
                }
                case RIGHT_FLOW: {

                    hugleftservo.setPosition(right_flow_positionL);
                    hugrightservo.setPosition(1-right_flow_positionR);

                    currentPosition = right_flow_positionL;
                    break;
                }
            }
        }
    }
}

