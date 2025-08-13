package org.firstinspires.ftc.teamcode.TeleOp.controllers;

import static org.firstinspires.ftc.teamcode.TeleOp.controllers.HangingController.hanging;
import static org.firstinspires.ftc.teamcode.TeleOp.controllers.HangingController.hangingCore;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.TeleOp.RobotMap;

@Config
public class DistanceSensorController {

    public enum distanceSensorStatus {
        ON,
        OFF
    }

    public static distanceSensorStatus currentStatus = distanceSensorStatus.OFF;
    public distanceSensorStatus previousStatus = null;
    public static DistanceSensor distance = null;

    public DistanceSensorController(RobotMap robot) {
        distance = robot.distance;
    }

    public void update() {
        double distancerope = 0; // Guardará la medición

        switch (currentStatus) {
            case ON:
                distancerope = distance.getDistance(DistanceUnit.CM);

                if (distancerope < 5) {
                    hangingCore.setPower(1);
                    hanging.setPower(1);
                } else {
                    hangingCore.setPower(0);
                    hanging.setPower(0);
                }
                break;

            case OFF:
                // Sensor apagado, no medimos distancia
                // Motores no se tocan aquí para que otra lógica los controle
                break;
        }
    }
}
