package TeleOp.controllers; // Ajusta la ruta si es necesario

import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.TeleOp.RobotMap;

public class IntakeRotationController {

    public enum RotationPosition {
        INIT,       // Posición inicial o de reposo/segura
        IZQUIERDA,    // Posición para recolectar elementos
        DERECHA,   // Posición intermedia para transferir
        HORIZONTAL// Posición para anotar bajo (ejemplo)

    }

    // Usaremos 'static' para currentStatus para mantener la consistencia con tu IntakeWristController
    // si así lo estás manejando desde MeteoritoTeleop.
    public static RotationPosition currentStatus = RotationPosition.INIT;
    private RotationPosition previousStatus = null;

    private Servo intakeRotationServo = null;

    // --- POSICIONES DEL SERVO (¡AJUSTA ESTOS VALORES! Son entre 0.0 y 1.0) ---
    public static double INIT_POSITION_SERVO = 0.22;  // Ejemplo: Centro
    public static double IZQUIERDA_POSITION_SERVO = 0;  // Ejemplo: Hacia un extremo
    public static double DERECHA_POSITION_SERVO = 0.5;  // Ejemplo: Intermedio 1
    public static double HORIZONTAL_POSITION_SERVO = 0.75;  // Ejemplo: Hacia el otro extremo

    public IntakeRotationController(RobotMap robot) {
        if (robot == null || robot.intakeRotationServo == null) { // <<< NOMBRE DEL SERVO EN ROBOTMAP
            System.err.println("IntakeRotationController Error: RobotMap o intakeRotationServo es NULL.");
             this.intakeRotationServo = null;
            return;
        }
        this.intakeRotationServo = robot.intakeRotationServo; // <<< ASIGNAR EL SERVO DESDE ROBOTMAP
        this.previousStatus = null; // Forzar la primera actualización

        // Opcional: si quieres que siempre inicie en INIT al crear la instancia del controlador
        IntakeRotationController.currentStatus = RotationPosition.INIT;
    }

    public void update() {
        if (intakeRotationServo == null) {
            System.err.println("IntakeRotationController.update(): intakeRotationServo es NULL. No se puede mover.");
            return;
        }

        if (currentStatus != previousStatus) {
            previousStatus = currentStatus; // Actualizar el estado previo

            double targetPositionServo = INIT_POSITION_SERVO; // Valor por defecto seguro

            switch (currentStatus) {
                case INIT:
                    targetPositionServo = INIT_POSITION_SERVO;
                    break;
                case IZQUIERDA:
                    targetPositionServo = IZQUIERDA_POSITION_SERVO;
                    break;
                case DERECHA:
                    targetPositionServo = DERECHA_POSITION_SERVO;
                    break;
                case HORIZONTAL:
                    targetPositionServo = HORIZONTAL_POSITION_SERVO;
                    break;
            }
            System.out.println("IntakeRotationController.update(): Moviendo servo a: " + targetPositionServo + " para estado: " + currentStatus);
            intakeRotationServo.setPosition(targetPositionServo);
        }
    }

    // Método de ayuda (opcional, pero útil si currentStatus no fuera static)
    public RotationPosition getCurrentStatus() {
        return currentStatus;
    }
}
// Si currentStatus no fuera static, necesitarías este setter
// public void setStatus(RotationPosition newStatus) {
//    currentStatus = newStatus;
// }
