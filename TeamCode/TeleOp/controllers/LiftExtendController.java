package TeleOp.controllers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.TeleOp.RobotMap; // Asegúrate de que RobotMap esté accesible

public class LiftExtendController {

    // --- Motores (directamente desde RobotMap) ---
    private DcMotorEx leftLiftExtendMotor = null;
    private DcMotorEx rightLiftExtendMotor = null;

    // Constante para la potencia máxima (puedes ajustarla)
    public static final double MAX_POWER = 0.8; // Evita usar 1.0 todo el tiempo para proteger los motores

    /**
     * Constructor
     * @param robotMap La instancia de RobotMap que contiene los motores inicializados.
     */
    public LiftExtendController(RobotMap robotMap) {
        if (robotMap != null) {
            this.leftLiftExtendMotor = robotMap.leftLiftExtendMotor;
            this.rightLiftExtendMotor = robotMap.rightLiftExtendMotor;

            // Asegurarse de que los motores estén en el modo correcto y con BRAKE
            if (this.leftLiftExtendMotor != null) {
                this.leftLiftExtendMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                this.leftLiftExtendMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            if (this.rightLiftExtendMotor != null) {
                this.rightLiftExtendMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                this.rightLiftExtendMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        } else {
            System.err.println("LiftExtendController Error: RobotMap es null. No se pueden inicializar motores.");
            // Considera lanzar una excepción o manejarlo de otra forma si RobotMap es crítico
        }
    }

    /**
     * Establece la potencia para ambos motores del LiftExtend.
     * @param power La potencia a aplicar. Positivo extiende, negativo retrae (dependiendo de la dirección del motor).
     *              Se limitará a +/- MAX_POWER.
     */
    public void setPower(double power) {
        if (leftLiftExtendMotor != null && rightLiftExtendMotor != null) {
            // Limitar la potencia para evitar valores excesivos
            double limitedPower = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));

            leftLiftExtendMotor.setPower(limitedPower);
            rightLiftExtendMotor.setPower(limitedPower);
        } else {
            // System.err.println("LiftExtendController: Motores no inicializados, no se puede establecer potencia.");
            // Puedes añadir telemetría o logging aquí si prefieres no usar System.err
        }
    }

    /**
     * Detiene ambos motores del LiftExtend.
     * Es útil para asegurar que se detengan cuando no se recibe entrada del joystick.
     */
    public void stopMotors() {
        if (leftLiftExtendMotor != null) {
            leftLiftExtendMotor.setPower(0);
        }
        if (rightLiftExtendMotor != null) {
            rightLiftExtendMotor.setPower(0);
        }
    }

    // Ya no necesitamos los métodos relacionados con encoders, posiciones o isAtTarget:
    // - update() (la lógica de estados se va)
    // - goToStatus()
    // - isAtTarget()
    // - getLeftMotorPosition(), getRightMotorPosition() (no son fiables sin encoders)
    // - constantes de posición (INIT_POSITION_TICKS, SCORE_LOW_TICKS, etc.)
    // - enum LiftExtendStatus
}