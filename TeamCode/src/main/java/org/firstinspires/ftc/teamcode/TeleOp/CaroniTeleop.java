package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.TeleOp.controllers.ExtendController;

@TeleOp(name = "CaroniTeleop", group = "Linear Opmode")
public class CaroniTeleop extends LinearOpMode {

    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;
    private RobotMap robot;
    private ExtendController extendController;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Inicializando Caroni...");
        telemetry.update();

        try {
            robot = new RobotMap(hardwareMap);
            extendController = new ExtendController(robot);
            telemetry.addData("RobotMap y ExtendController", "Inicializados.");
        } catch (Exception e) {
            telemetry.addData("ERROR CRÍTICO", "Fallo al inicializar RobotMap o ExtendController: " + e.getMessage());
            telemetry.update();
            sleep(5000);
            requestOpModeStop();
            return;
        }

        try {
            leftMotor = hardwareMap.get(DcMotor.class, "izquierda");
            rightMotor = hardwareMap.get(DcMotor.class, "derecha");

            leftMotor.setDirection(DcMotor.Direction.FORWARD);
            rightMotor.setDirection(DcMotor.Direction.REVERSE);

            leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            telemetry.addData("Status", "Caroni Inicializado. ¡Listo para la acción!");
        } catch (Exception e) {
            telemetry.addData("Status", "Error al inicializar motores: " + e.getMessage());
            telemetry.update();
            sleep(5000);
            requestOpModeStop();
            return;
        }

        telemetry.update();

        waitForStart();

        // Estado inicial seguro al comenzar
        if (extendController != null) {
            extendController.goToPosition(ExtendController.ExtendPosition.CERRADO);
            extendController.update();
        }

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            // --- Control del chasis ---
            double drivePower = -gamepad1.left_stick_y;
            double turnPower = gamepad1.right_stick_x;

            double leftMotorPower = drivePower + turnPower;
            double rightMotorPower = drivePower - turnPower;

            double max = Math.max(Math.abs(leftMotorPower), Math.abs(rightMotorPower));
            if (max > 1.0) {
                leftMotorPower /= max;
                rightMotorPower /= max;
            }

            if (leftMotor != null) {
                leftMotor.setPower(leftMotorPower);
            }
            if (rightMotor != null) {
                rightMotor.setPower(rightMotorPower);
            }
            // --- Control del sistema de extensión con FSM ---
            if (extendController != null) {
                // A/Cross button: alternar entre ABIERTO y CERRADO
                if (currentGamepad1.cross && !previousGamepad1.cross) {
                    if (ExtendController.currentState == ExtendController.ExtendPosition.ABIERTO) {
                        extendController.goToPosition(ExtendController.ExtendPosition.CERRADO);
                    } else {
                        extendController.goToPosition(ExtendController.ExtendPosition.ABIERTO);
                    }
                }

                // B/Circle button: ir a MEDIO
                if (currentGamepad1.circle && !previousGamepad1.circle) {
                    extendController.goToPosition(ExtendController.ExtendPosition.MEDIO);
                }

                // Actualizar FSM cada ciclo
                extendController.update();
            }

            // --- Telemetría ---
            telemetry.addData("Estado", "Corriendo");
            telemetry.addData("Chasis Drive", "%.2f", drivePower);
            telemetry.addData("Chasis Turn", "%.2f", turnPower);
            telemetry.addData("Chasis LPower", "%.2f", leftMotorPower);
            telemetry.addData("Chasis RPower", "%.2f", rightMotorPower);

            if (extendController != null) {
                telemetry.addData("--- Extension ---", "");
                telemetry.addData("Ext Desired State", ExtendController.currentState);
                telemetry.addData("Ext Left Ticks", extendController.getCurrentState());
                telemetry.addData("Ext Right Ticks", extendController.getCurrentState());
                telemetry.addData("Ext Motors Busy", extendController.areMotorsBusy());
            }

            telemetry.update();
        }

        // --- Detener motores al finalizar ---
        if (leftMotor != null) leftMotor.setPower(0);
        if (rightMotor != null) rightMotor.setPower(0);
        if (extendController != null) extendController.stopMotors();

        telemetry.addData("Status", "Caroni Detenido.");
        telemetry.update();
    }
}
