package TeleOp.controllers;

import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.TeleOp.RobotMap;

public class IntakeClawController {

    public double getServoTargetPosition() {
        return 0;
    }

    // --- Enum para los estados/posiciones de la Garra (Claw) ---
    public enum ClawPosition {
        OPEN,
        CLOSE
        // No hay estado INIT explícito aquí, el estado inicial se manejará en TeleOp o por defecto
    }

    // --- Constantes de Posición del Servo ---
    // ¡¡AJUSTA ESTOS VALORES SEGÚN TU ROBOT!!
    public static final double OPEN_POSITION_SERVO = 0.9;  // Ejemplo: Garra completamente abierta
    public static final double CLOSE_POSITION_SERVO = 0.5; // Ejemplo: Garra completamente cerrada
    // No se necesita DEFAULT_POSITION_SERVO si siempre se parte de OPEN o CLOSE

    // --- Variables Miembro ---
    private Servo intakeClawServo;
    // 'currentStatus' será estático, como en tus otros controladores.
    // El estado inicial puede ser definido en TeleOp o por defecto aquí.
    public static ClawPosition currentStatus = ClawPosition.CLOSE; // Por ejemplo, empezar cerrado
    private ClawPosition previousStatus = currentStatus; // Sincronizar al inicio

    // --- Constructor ---
    public IntakeClawController(RobotMap robotMap) {
        if (robotMap == null) {
            System.err.println("IntakeClawController Error: RobotMap es NULL.");
            return;
        }
        this.intakeClawServo = robotMap.intakeClawServo; // Asume que 'intakeClawServo' existe en RobotMap

        if (this.intakeClawServo == null) {
            System.err.println("IntakeClawController Error: robotMap.intakeClawServo es NULL. Verifica RobotMap y la configuración del hardware.");
            return;
        }
        System.out.println("IntakeClawController: Creado y servo obtenido. Estado inicial: " + currentStatus);
        // Aplicar la posición inicial al crear el controlador si se desea asegurar
        // update(); // Opcional, si quieres que se mueva a 'currentStatus' inmediatamente al instanciar.
        // TeleOp lo llamará en el primer ciclo de todas formas.
    }

    // --- Método de Actualización (se llama en el bucle de TeleOp) ---
    public void update() {
        if (intakeClawServo == null) {
            return; // No hacer nada si el servo no está inicializado
        }

        // Solo para depuración, puedes quitar esto si no lo necesitas
        if (currentStatus != previousStatus) {
            System.out.println("IntakeClawController: Cambiando estado de " + previousStatus + " a " + currentStatus);
            previousStatus = currentStatus;
        }

        switch (currentStatus) {
            case OPEN:
                intakeClawServo.setPosition(OPEN_POSITION_SERVO);
                break;
            case CLOSE:
                intakeClawServo.setPosition(CLOSE_POSITION_SERVO);
                break;
            // No hay caso default explícito necesario si el enum solo tiene OPEN/CLOSE
            // y currentStatus siempre es uno de ellos.
        }
    }

    // --- Métodos Adicionales (Opcionales, pero útiles para telemetría) ---
    public ClawPosition getCurrentStatus() { // Getter no estático para la instancia
        return currentStatus; // Devuelve el estado estático
    }





    public void disable() {
        if (intakeClawServo != null && intakeClawServo.getController() != null) {
            intakeClawServo.getController().pwmDisable();
            System.out.println("IntakeClawController: Servo deshabilitado (PWM off).");
        }
    }

    public void enable() {
        if (intakeClawServo != null && intakeClawServo.getController() != null) {
            intakeClawServo.getController().pwmEnable();
            update(); // Aplicar la posición actual
            System.out.println("IntakeClawController: Servo habilitado (PWM on).");
        }
    }
}