package org.firstinspires.ftc.teamcode.TeleOp.controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.TeleOp.RobotMap;

import static org.firstinspires.ftc.teamcode.TeleOp.controllers.HangingController.hanging;
import static org.firstinspires.ftc.teamcode.TeleOp.controllers.HangingController.hangingCore;

@Config
public class MultiSensorController {

    public enum sensorStatus {
        ON,
        OFF
    }

    public static sensorStatus currentStatus = sensorStatus.OFF;
    private sensorStatus previousStatus = null;

    public static DistanceSensor distance;
    public static RevColorSensorV3 colorLeft;
    public static RevColorSensorV3 colorRight;

    public static double distancerope;
    public static double leftProximity;
    public static double rightProximity;

    public static int leftRed, leftGreen, leftBlue;
    public static int rightRed, rightGreen, rightBlue;

    // Parámetros ajustables desde FTC Dashboard
    public static int whiteThreshold = 200;      // Brillo mínimo
    public static int diffThreshold = 30;        // Diferencia máxima entre canales

    public MultiSensorController(RobotMap robot) {
        distance = robot.distance;
        colorLeft = robot.colorLeft;
        colorRight = robot.colorRight;
    }
    // Dentro de MultiSensorController
    public static boolean sensorTriggered = false; // nuevo estado

    public void update() {
        if (currentStatus != previousStatus) {
            previousStatus = currentStatus;
        }

        switch (currentStatus) {
            case ON:
                // Actualizar distancias y colores
                distancerope = distance.getDistance(DistanceUnit.CM);

                leftRed = colorLeft.red();
                leftGreen = colorLeft.green();
                leftBlue = colorLeft.blue();
                leftProximity = colorLeft.getDistance(DistanceUnit.CM);

                rightRed = colorRight.red();
                rightGreen = colorRight.green();
                rightBlue = colorRight.blue();
                rightProximity = colorRight.getDistance(DistanceUnit.CM);

                // Si detecta condición y aún no estaba activado
                if (!sensorTriggered &&
                        distancerope < 5 &&
                        (isWhite(leftRed, leftGreen, leftBlue) || isWhite(rightRed, rightGreen, rightBlue))) {
                    sensorTriggered = true; // se queda activado
                }

                // Mantener encendido si está activado
                if (sensorTriggered) {
                    hangingCore.setPower(1);
                    hanging.setPower(1);
                } else {
                    hangingCore.setPower(0);
                    hanging.setPower(0);
                }
                break;

            case OFF:
                sensorTriggered = false; // al apagar desde circle, resetea
                hangingCore.setPower(0);
                hanging.setPower(0);
                break;
        }
    }

    // Detección de blanco
    public static boolean isWhite(int r, int g, int b) {
        int threshold = 200;
        int diffThreshold = 30;
        return (r > threshold && g > threshold && b > threshold) &&
                (Math.abs(r - g) < diffThreshold &&
                        Math.abs(r - b) < diffThreshold &&
                        Math.abs(g - b) < diffThreshold);
    }
}
