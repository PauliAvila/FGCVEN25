package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple; // Para setDirection
import com.qualcomm.robotcore.util.Range; // Para recortar valores
import com.qualcomm.robotcore.hardware.Gamepad; // Para edge detection
import org.firstinspires.ftc.teamcode.TeleOp.RobotMap;
import org.firstinspires.ftc.teamcode.TeleOp.controllers.ExtendController;
@TeleOp(name = "CaroniTeleop", group = "Linear Opmode")
public class CaroniTeleop extends LinearOpMode {

    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;
    private RobotMap robot;
    private ExtendController extendController;
    // Para detección de flanco (edge detection) en botones del gamepad
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Inicializando Caroni...");
        telemetry.update();
        // --- INICIALIZACIÓN DE ROBOTMAP Y CONTROLADORES ---
        try {
            robot = new RobotMap(hardwareMap); // Esto debería inicializar los motores de extensión en RobotMap
            extendController = new ExtendController(robot); // El controlador configura los motores de extensión
            telemetry.addData("RobotMap y ExtendController", "Inicializados.");
        } catch (Exception e) {
            telemetry.addData("ERROR CRÍTICO", "Fallo al inicializar RobotMap o ExtendController: " + e.getMessage());
            telemetry.update();
            sleep(5000);
            requestOpModeStop();
            return;
        }
        // --- INICIALIZACIÓN DE MOTORES ---
        try {
            leftMotor = hardwareMap.get(DcMotor.class, "izquierda"); // Usa el nombre de tu configuración
            rightMotor = hardwareMap.get(DcMotor.class, "derecha"); // Usa el nombre de tu configuración


            leftMotor.setDirection(DcMotor.Direction.FORWARD);
            rightMotor.setDirection(DcMotor.Direction.REVERSE);

            // Establecer el modo de los motores (RUN_WITHOUT_ENCODER es común para control directo en TeleOp)
            leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // Establecer el comportamiento cuando la potencia es cero (BRAKE es bueno para que no se deslice)
            leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            telemetry.addData("Status", "Caroni Inicializado. ¡Listo para la acción!");
        } catch (Exception e) {
            telemetry.addData("Status", "Error al inicializar motores: " + e.getMessage());
            telemetry.update();
            // Considera detener el OpMode si la inicialización falla críticamente
            sleep(5000); // Dar tiempo para leer el error
            requestOpModeStop();
            return;
        }
        telemetry.update();

        waitForStart();

        // Asegurarse de que la extensión esté en el estado inicial deseado al comenzar
        if (extendController != null) {
            ExtendController.currentTargetState = ExtendController.ExtendPosition.CERRADO;
            extendController.update(); // Llamar una vez para que comience a ir a la posición inicial
        }


        while (opModeIsActive()) {
            // --- ACTUALIZACIÓN DE ESTADO DE GAMEPADS (para edge detection) ---
            try {
                previousGamepad1.copy(currentGamepad1);
                currentGamepad1.copy(gamepad1);

                // Si usas gamepad2:
                // previousGamepad2.copy(currentGamepad2);
                // currentGamepad2.copy(gamepad2);
            } catch (Exception e) {
                // Manejar error de copia de gamepad si ocurre
                telemetry.addData("Gamepad Error", e.getMessage());
            }


            // --- CONTROL DEL CHASIS

            // --- LECTURA DE JOYSTICKS ---
            // Joystick izquierdo Y para adelante/atrás
            // Es negativo cuando se empuja hacia arriba, así que lo invertimos.
            double drivePower = -gamepad1.left_stick_y;

            // Joystick derecho X para girar
            double turnPower = gamepad1.right_stick_x;

            // --- CÁLCULO DE POTENCIA PARA CADA MOTOR ---
            double leftMotorPower = drivePower + turnPower;
            double rightMotorPower = drivePower - turnPower;

            // --- NORMALIZACIÓN/RECORTE DE POTENCIA ---
            // Asegura que la potencia no exceda +/- 1.0
            // Opción 1: Normalización si alguna potencia excede 1.0
            double max = Math.max(Math.abs(leftMotorPower), Math.abs(rightMotorPower));
            if (max > 1.0) {
                leftMotorPower /= max;
                rightMotorPower /= max;
            }

            // --- APLICAR POTENCIA A LOS MOTORES ---
            if (leftMotor != null) {
                leftMotor.setPower(leftMotorPower);
            }
            if (rightMotor != null) {
                rightMotor.setPower(rightMotorPower);
            }

            // --- CONTROL DEL SISTEMA DE EXTENSIÓN ---
            if (extendController != null) {
//Cross en PS) para alternar ESTRICTAMENTE entre ABIERTO y CERRADO
                if (currentGamepad1.cross && !previousGamepad1.cross) { // Botón A/Cross (PS) presionado
                    telemetry.addLine("BOTON CROSS PRESIONADO - Intentando alternar ABIERTO/CERRADO");
                    if (ExtendController.currentTargetState == ExtendController.ExtendPosition.ABIERTO) {
                        ExtendController.currentTargetState = ExtendController.ExtendPosition.CERRADO;
                        telemetry.addLine("Alternando a CERRADO");
                    } else {
                        // Si NO está ABIERTO (puede ser CERRADO o MEDIO), lo mandamos a ABIERTO.
                        // Esto asegura que siempre alterne hacia ABIERTO si no estaba ya ABIERTO.
                        ExtendController.currentTargetState = ExtendController.ExtendPosition.ABIERTO;
                        telemetry.addLine("Alternando a ABIERTO (desde CERRADO o MEDIO)");
                    }
                }

                // 2. Control con el botón "B" (Circle en PS) para ir a MEDIO
                if (currentGamepad1.circle && !previousGamepad1.circle) {
                    ExtendController.currentTargetState = ExtendController.ExtendPosition.MEDIO;
                    telemetry.addLine("BOTON B/CIRCLE PRESIONADO - Estado: MEDIO");
                }

                extendController.update();
            }





            // --- TELEMETRÍA ---
                telemetry.addData("Estado", "Corriendo");
                telemetry.addData("Chasis Drive", "%.2f", drivePower);
                telemetry.addData("Chasis Turn", "%.2f", turnPower);
                telemetry.addData("Chasis LPower", "%.2f", leftMotorPower);
                telemetry.addData("Chasis RPower", "%.2f", rightMotorPower);

                if (extendController != null) {
                    telemetry.addData("--- Extension ---", "");
                    telemetry.addData("Ext Desired State", ExtendController.currentTargetState);
                    telemetry.addData("Ext Left Ticks", extendController.getLeftMotorPosition());
                    telemetry.addData("Ext Right Ticks", extendController.getRightMotorPosition());
                    telemetry.addData("Ext Motors Busy", extendController.areMotorsBusy());
                } else {
                    telemetry.addLine("ExtendController NO INICIALIZADO.");
                }

                telemetry.update();
            }

            // --- DETENER MOTORES CUANDO EL OpMode TERMINA ---
            if (leftMotor != null) {
                leftMotor.setPower(0);
            }
            if (rightMotor != null) {
                rightMotor.setPower(0);
            }
            telemetry.addData("Status", "Caroni Detenido.");
            telemetry.update();
        }
    }


