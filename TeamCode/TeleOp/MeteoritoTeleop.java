package org.firstinspires.ftc.teamcode.TeleOp;

// No necesitas android.drm.DrmStore
import static org.firstinspires.ftc.teamcode.TeleOp.controllers.IntakeRotationController.RotationPosition.INIT;

// import android.view.View; // No parece usarse

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
// No necesitas Servo aquí si todos los servos se manejan a través de controladores
import com.qualcomm.robotcore.util.ElapsedTime;

// Importar los controladores
import org.firstinspires.ftc.teamcode.TeleOp.controllers.IntakeRotationController;
import org.firstinspires.ftc.teamcode.TeleOp.controllers.LiftController;
import org.firstinspires.ftc.teamcode.TeleOp.controllers.LiftExtendController; // Asegúrate que es la versión modificada para control manual
import org.firstinspires.ftc.teamcode.TeleOp.controllers.IntakeWristController;
import org.firstinspires.ftc.teamcode.TeleOp.controllers.IntakeClawController;

@Config
@TeleOp(name="MeteoritoTeleop", group="Linear OpMode") // Nombre ligeramente cambiado
public class MeteoritoTeleop extends LinearOpMode {

    // --- MIEMBROS DEL OPMODE ---
    private final ElapsedTime runtime = new ElapsedTime();

    // Motores del Chasis (Drivetrain)
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx rightBackDrive = null;

    // --- Controladores ---
    private RobotMap robot;
    private LiftController liftController;
    private LiftExtendController liftExtendController; // Para control manual
    private IntakeWristController intakeWristController;
    private IntakeRotationController intakeRotationController;
    private IntakeClawController intakeClawController;

    private boolean crossActionToggleState = false;

    // Gamepads para detección de flancos (edge detection)
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    // --- Variables para el control manual del LiftExtend ---
    private double liftExtendJoystickValue = 0;
    private double liftExtendPowerToApply = 0;


    // --- Métodos de Ayuda para Chasis ---
    public void setMotorRunningMode(DcMotor.RunMode runningMode) {
        if (leftFrontDrive != null) leftFrontDrive.setMode(runningMode);
        if (rightFrontDrive != null) rightFrontDrive.setMode(runningMode);
        if (leftBackDrive != null) leftBackDrive.setMode(runningMode);
        if (rightBackDrive != null) rightBackDrive.setMode(runningMode);
    }

    public void setMotorZeroPowerBehaviour(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        if (leftFrontDrive != null) leftFrontDrive.setZeroPowerBehavior(zeroPowerBehavior);
        if (rightFrontDrive != null) rightFrontDrive.setZeroPowerBehavior(zeroPowerBehavior);
        if (leftBackDrive != null) leftBackDrive.setZeroPowerBehavior(zeroPowerBehavior);
        if (rightBackDrive != null) rightBackDrive.setZeroPowerBehavior(zeroPowerBehavior);
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "OpMode Initializing...");
        telemetry.update();

        // --- INICIALIZACIÓN DE HARDWARE Y CONTROLADORES ---
        try {
            robot = new RobotMap(hardwareMap);
            telemetry.addData("RobotMap", "Instancia creada.");

            leftFrontDrive = hardwareMap.get(DcMotorEx.class, "leftFront");
            leftBackDrive = hardwareMap.get(DcMotorEx.class, "leftBack");
            rightFrontDrive = hardwareMap.get(DcMotorEx.class, "rightFront");
            rightBackDrive = hardwareMap.get(DcMotorEx.class, "rightBack");

            leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
            rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

            setMotorRunningMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            setMotorZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);

            telemetry.addData("Chassis", "Motores inicializados.");

        } catch (Exception e) {
            telemetry.addData("ERROR CRÍTICO", "Fallo en inicialización de Chasis o RobotMap: " + e.getMessage());
            telemetry.update();
            sleep(5000); // Dar tiempo para leer el mensaje
            requestOpModeStop();
            return;
        }

        try {
            liftController = new LiftController(robot);
            telemetry.addData("LiftController", "Creado.");
        } catch (Exception e) {
            telemetry.addData("ERROR CRÍTICO", "Fallo al crear LiftController: " + e.getMessage());
            telemetry.update();
            sleep(5000);
            requestOpModeStop();
            return;
        }

        try {
            // Asegúrate de que LiftExtendController está modificado para control manual
            // y no depende de encoders para su lógica principal (usa setPower).
            liftExtendController = new LiftExtendController(robot);
            telemetry.addData("LiftExtendController", "Creado (Modo Manual Esperado).");
        } catch (Exception e) {
            telemetry.addData("ERROR CRÍTICO", "Fallo al crear LiftExtendController: " + e.getMessage());
            telemetry.update();
            sleep(5000);
            requestOpModeStop();
            return;
        }

        try {
            this.intakeWristController = new IntakeWristController(robot);
            telemetry.addData("IntakeWristController", "Creado.");
        } catch (Exception e) {
            telemetry.addData("ERROR CRÍTICO", "Fallo al crear IntakeWristController: " + e.getMessage());
            telemetry.update();
            sleep(5000);
            requestOpModeStop();
            return;
        }

        try {
            this.intakeRotationController = new IntakeRotationController(robot);
            telemetry.addData("IntakeRotationCtrl", "Creado.");
        } catch (Exception e) {
            telemetry.addData("ERROR CRÍTICO", "Fallo al crear IntakeRotationController: " + e.getMessage());
            telemetry.update();
            sleep(5000);
            requestOpModeStop();
            return;
        }
        try {
            this.intakeClawController = new IntakeClawController(robot);
            telemetry.addData("IntakeClawController", "Creado.");
        } catch (Exception e) {
            telemetry.addData("ERROR CRÍTICO", "Fallo al crear IntakeClawController: " + e.getMessage());
            telemetry.update();
            sleep(5000);
            requestOpModeStop();
            return;
        }

        // Estado inicial de la garra
        if (intakeClawController != null) { // Verificar nulidad
            IntakeClawController.currentStatus = IntakeClawController.ClawPosition.CLOSE;
        }


        telemetry.addData("Status", "Initialized. Press START.");
        if (this.intakeWristController != null) {
            telemetry.addData("IntakeWrist Initial Status", IntakeWristController.currentStatus);
        }
        if (this.intakeRotationController != null) {
            telemetry.addData("IntakeRotation Initial Status", IntakeRotationController.currentStatus);
        }
        if (this.liftController != null) {
            telemetry.addData("Lift Initial Status", LiftController.currentStatus);
        }
        telemetry.update();

        // Detener LiftExtend al inicio como precaución
        if (liftExtendController != null) {
            liftExtendController.stopMotors();
        }

        waitForStart();
        runtime.reset();

        if (isStopRequested()) return;

        // --- BUCLE PRINCIPAL DEL TELEOP ---
        while (opModeIsActive() && !isStopRequested()) {

            // --- ACTUALIZACIÓN DE GAMEPADS (MUY IMPORTANTE) ---
            try {
                previousGamepad1.copy(currentGamepad1);
                previousGamepad2.copy(currentGamepad2);
                currentGamepad1.copy(gamepad1);
                currentGamepad2.copy(gamepad2);
            } catch (Exception e) {
                telemetry.addData("Gamepad Error", "Error copying gamepad state: " + e.getMessage());
                // Considera cómo manejar esto, podría ser crítico
            }

            // === CONTROL DEL LIFT EXTEND (MANUAL SIN ENCODERS) ===
            if (liftExtendController != null) {
                // Usaremos, por ejemplo, el joystick derecho del gamepad2 para la extensión del lift.
                // gamepad2.right_stick_y es negativo hacia arriba y positivo hacia abajo.
                // Si quieres que "arriba" en el joystick extienda, invierte el signo.
                liftExtendJoystickValue = -gamepad2.right_stick_y; // Invierte para que arriba en el joystick = extender

                // Aplicar una "deadzone" (zona muerta)
                if (Math.abs(liftExtendJoystickValue) < 0.1) { // Ajusta la deadzone si es necesario
                    liftExtendPowerToApply = 0;
                } else {
                    liftExtendPowerToApply = liftExtendJoystickValue;
                    // Opcional: escalar la potencia si se desea un control más fino o más rápido
                    // liftExtendPowerToApply = Math.signum(liftExtendJoystickValue) * Math.pow(Math.abs(liftExtendJoystickValue), 2); // Ejemplo de curva cuadrática
                }
                liftExtendController.setPower(liftExtendPowerToApply);
            }


            // --- LÓGICA DEL LIFT VERTICAL (LiftController) ---
            if (currentGamepad2.cross && !previousGamepad2.cross) {
                crossActionToggleState = !crossActionToggleState;
                if (crossActionToggleState) {
                    if (this.intakeWristController != null) IntakeWristController.currentStatus = IntakeWristController.WristPosition.SCORE_MID;
                    // Aquí podrías añadir también una acción para el LjniftController si es parte de la secuencia "HIGH/SCORE_MID"
                    // if (liftController != null) LiftController.currentStatus = LiftController.LiftStatus.HIGH; // Ejemplo
                } else {
                    if (this.intakeWristController != null) IntakeWristController.currentStatus = IntakeWristController.WristPosition.INIT;
                    // if (liftController != null) LiftController.currentStatus = LiftController.LiftStatus.INIT; // Ejemplo
                }
            }

            if (currentGamepad2.circle && !previousGamepad2.circle) {
                if (liftController != null) { // Verificar nulidad
                    if (LiftController.currentStatus == LiftController.LiftStatus.MEDIUM) {
                        LiftController.currentStatus = LiftController.LiftStatus.INIT;
                    } else {
                        LiftController.currentStatus = LiftController.LiftStatus.MEDIUM;
                    }
                }
            }

            // --- LÓGICA DEL INTAKE WRIST (IntakeWristController) ---
            if (this.intakeWristController != null) {
                if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {
                    switch (IntakeWristController.currentStatus) {
                        case INIT: IntakeWristController.currentStatus = IntakeWristController.WristPosition.COLLECT; break;
                        case COLLECT: IntakeWristController.currentStatus = IntakeWristController.WristPosition.GRAB; break;
                        case GRAB: IntakeWristController.currentStatus = IntakeWristController.WristPosition.SCORE_MID; break;
                        case SCORE_MID: IntakeWristController.currentStatus = IntakeWristController.WristPosition.INIT; break;
                    }
                }
            }

            // --- LÓGICA DEL INTAKE ROTATION (IntakeRotationController) ---
            if (this.intakeRotationController != null) {
                if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {
                    IntakeRotationController.currentStatus = INIT;
                } else if (currentGamepad2.dpad_left && !previousGamepad2.dpad_left) {
                    IntakeRotationController.currentStatus = IntakeRotationController.RotationPosition.IZQUIERDA;
                } else if (currentGamepad2.dpad_right && !previousGamepad2.dpad_right) {
                    if (IntakeRotationController.currentStatus == IntakeRotationController.RotationPosition.DERECHA) {
                        IntakeRotationController.currentStatus = IntakeRotationController.RotationPosition.HORIZONTAL;
                    } else {
                        IntakeRotationController.currentStatus = IntakeRotationController.RotationPosition.DERECHA;
                    }
                }
            }

            // --- LÓGICA DEL INTAKE CLAW (IntakeClawController) ---
            if (this.intakeClawController != null) {
                if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
                    if (IntakeClawController.currentStatus == IntakeClawController.ClawPosition.OPEN) {
                        IntakeClawController.currentStatus = IntakeClawController.ClawPosition.CLOSE;
                    } else {
                        IntakeClawController.currentStatus = IntakeClawController.ClawPosition.OPEN;
                    }
                }
            }

            // --- ACTUALIZAR TODOS LOS CONTROLADORES ---
            // El LiftExtendController para control manual no necesita un update() si solo usa setPower().
            // Si tus otros controladores (servos, motores con RUN_TO_POSITION) tienen un método update(), llámalos.
            if (liftController != null) liftController.update();
            if (this.intakeWristController != null) this.intakeWristController.update();
            if (this.intakeRotationController != null) this.intakeRotationController.update();
            if (this.intakeClawController != null) this.intakeClawController.update();
            // NO necesitas llamar a liftExtendController.update() si es la versión manual.

            // --- DRIVETRAIN (Chasis) ---
            // (Tu lógica de drivetrain existente, que parece correcta para mecanum)
            double lateral = -gamepad1.left_stick_x; // Usar - si es necesario para tu robot
            double axial = -gamepad1.left_stick_y;   // Invertir Y si empujar hacia adelante da negativo
            double yaw = -gamepad1.right_stick_x; // Usar - si es necesario

            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            if (leftFrontDrive != null) leftFrontDrive.setPower(leftFrontPower);
            if (rightFrontDrive != null) rightFrontDrive.setPower(rightFrontPower);
            if (leftBackDrive != null) leftBackDrive.setPower(leftBackPower);
            if (rightBackDrive != null) rightBackDrive.setPower(rightBackPower);

            // --- TELEMETRÍA ---
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            telemetry.addData("--- Lift Vertical ---", "");
            if (liftController != null && robot.leftLift != null) { // Asumo que robot.leftLift es el motor principal del lift
                telemetry.addData("Lift Status", LiftController.currentStatus);
                // Descomenta si quieres ver los detalles del motor y LiftController usa RUN_TO_POSITION
                // telemetry.addData("Lift Target Ticks", robot.leftLift.getTargetPosition());
                // telemetry.addData("Lift Actual Ticks", robot.leftLift.getCurrentPosition());
            }

            telemetry.addData("--- Lift Extensión (Manual) ---", "");
            telemetry.addData("LiftExtend Joystick Raw", "%.2f", liftExtendJoystickValue);
            telemetry.addData("LiftExtend Power Aplicada", "%.2f", liftExtendPowerToApply);
            if (liftExtendController == null) {
                telemetry.addLine("LiftExtendController NO INICIALIZADO");
            }
            // Ya no tiene sentido la telemetría de Target/Actual/Busy para LiftExtend si es manual

            telemetry.addData("--- Intake Wrist ---", "");
            if (this.intakeWristController != null && robot.intakeWristServo != null) {
                telemetry.addData("Wrist Status", IntakeWristController.currentStatus);
                // telemetry.addData("Wrist Actual Servo Pos", "%.3f", robot.intakeWristServo.getPosition());
            }

            telemetry.addData("--- Intake Rotation ---", "");
            if (this.intakeRotationController != null && robot.intakeRotationServo != null) {
                telemetry.addData("Rotation Status", IntakeRotationController.currentStatus);
                // telemetry.addData("Rotation Actual Servo Pos", "%.3f", robot.intakeRotationServo.getPosition());
            }

            telemetry.addData("--- Intake Claw ---", "");
            if (this.intakeClawController != null && robot.intakeClawServo != null) {
                telemetry.addData("Claw Status", IntakeClawController.currentStatus);
                // telemetry.addData("Claw Actual Servo Pos", "%.3f", robot.intakeClawServo.getPosition());
            }
            telemetry.update();
        }

        // --- DETENER MOTORES AL FINALIZAR EL OpMode ---
        if (leftFrontDrive != null) leftFrontDrive.setPower(0);
        if (rightFrontDrive != null) rightFrontDrive.setPower(0);
        if (leftBackDrive != null) leftBackDrive.setPower(0);
        if (rightBackDrive != null) rightBackDrive.setPower(0);

        if (liftController != null && robot.leftLift != null) { // Asumo que LiftController podría tener un método para detener su motor
            robot.leftLift.setPower(0); // O liftController.stopMotor();
        }
        if (liftExtendController != null) {
            liftExtendController.stopMotors(); // Importante para el control manual
        }

        // Para los servos, normalmente no es necesario "detenerlos" activamente al final
        // a menos que tengas lógica específica para desactivar el pulso del servo (menos común).
        telemetry.addData("Status", "OpMode Stopped. Motors should be off.");
        telemetry.update();
    }
}