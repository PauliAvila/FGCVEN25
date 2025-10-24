package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import android.graphics.Color;

@TeleOp(name = "ColorSensor Calibración HSV", group = "Sensor")
public class ColorSensorTest extends LinearOpMode {

    ColorSensor colorSensor;

    // ---- Rangos de Hue configurables ----
    final float[] RED_HUE_RANGE = {0f, 30f};
    final float[] ORANGE_HUE_RANGE = {34f, 70f};
    final float[] YELLOW_HUE_RANGE = {80f, 100f};
    final float[] BLUE_HUE_RANGE = {200f, 250f};

    // ---- Parámetros de saturación y brillo ----
    final float MIN_SATURATION = 0.45f;
    final float MIN_VALUE = 0.35f;

    @Override
    public void runOpMode() {

        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        telemetry.addLine("📊 Iniciando sensor de color...");
        telemetry.addLine("Mantén el sensor frente al color que deseas calibrar.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Obtener HSV
            float[] hsv = new float[3];
            Color.RGBToHSV(
                    colorSensor.red() * 8,
                    colorSensor.green() * 8,
                    colorSensor.blue() * 8,
                    hsv
            );

            float hue = hsv[0];
            float sat = hsv[1];
            float val = hsv[2];

            // ---- Mostrar datos en tiempo real ----
            telemetry.addLine("🎨 VALORES EN VIVO");
            telemetry.addData("Hue", "%.1f°", hue);
            telemetry.addData("Saturación", "%.2f", sat);
            telemetry.addData("Valor (brillo)", "%.2f", val);
            telemetry.addLine();

            // ---- Mostrar rangos configurados ----
            telemetry.addLine("🎯 RANGOS CONFIGURADOS:");
            telemetry.addData("Rojo", "[%.0f° - %.0f°]", RED_HUE_RANGE[0], RED_HUE_RANGE[1]);
            telemetry.addData("Naranja", "[%.0f° - %.0f°]", ORANGE_HUE_RANGE[0], ORANGE_HUE_RANGE[1]);
            telemetry.addData("Amarillo", "[%.0f° - %.0f°]", YELLOW_HUE_RANGE[0], YELLOW_HUE_RANGE[1]);
            telemetry.addData("Azul", "[%.0f° - %.0f°]", BLUE_HUE_RANGE[0], BLUE_HUE_RANGE[1]);
            telemetry.addData("Mín. Saturación", "%.2f", MIN_SATURATION);
            telemetry.addData("Mín. Valor", "%.2f", MIN_VALUE);
            telemetry.addLine();

            // ---- Detectar color actual ----
            if (isRed(hsv)) {
                telemetry.addLine("🔴 Detectado: ROJO");
            } else if (isOrange(hsv)) {
                telemetry.addLine("🟧 Detectado: NARANJA");
            } else if (isYellow(hsv)) {
                telemetry.addLine("🟨 Detectado: AMARILLO");
            } else if (isBlue(hsv)) {
                telemetry.addLine("🔵 Detectado: AZUL");
            } else {
                telemetry.addLine("⚪ Ningún color reconocido");
            }

            telemetry.update();
        }
    }

    // ---- Métodos de detección ----

    boolean isRed(float[] hsv) {
        return ((hsv[0] >= RED_HUE_RANGE[0] && hsv[0] <= RED_HUE_RANGE[1]) ||
                (hsv[0] >= 350 && hsv[0] <= 360)) &&
                hsv[1] >= MIN_SATURATION && hsv[2] >= MIN_VALUE;
    }

    boolean isOrange(float[] hsv) {
        return (hsv[0] >= ORANGE_HUE_RANGE[0] && hsv[0] <= ORANGE_HUE_RANGE[1]) &&
                hsv[1] >= MIN_SATURATION && hsv[2] >= MIN_VALUE;
    }

    boolean isYellow(float[] hsv) {
        return (hsv[0] >= YELLOW_HUE_RANGE[0] && hsv[0] <= YELLOW_HUE_RANGE[1]) &&
                hsv[1] >= MIN_SATURATION && hsv[2] >= MIN_VALUE;
    }

    boolean isBlue(float[] hsv) {
        return (hsv[0] >= BLUE_HUE_RANGE[0] && hsv[0] <= BLUE_HUE_RANGE[1]) &&
                hsv[1] >= MIN_SATURATION && hsv[2] >= MIN_VALUE;
    }
}
