package TeleOp.controllers; // Asegúrate de la ruta

import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.TeleOp.RobotMap;
// import com.acmerobotics.dashboard.config.Config; // Si usas FTC Dashboard

// @Config // Si usas FTC Dashboard
public class IntakeWristController {

    public static LiftController.LiftStatus ServoRotationStatus;

    public enum WristPosition {
        COLLECT, // Posición inicial o de reposo
        GRAB, // Posición para agarrar
        INIT, SCORE_MID // <<<--- NUEVA TERCERA POSICIÓN (ejemplo)
    }

    // Usar 'static' si quieres controlar este estado globalmente desde el TeleOp
    // y que persista si recreas la instancia del controlador (no recomendado generalmente)
    // O quitar 'static' si cada instancia del controlador maneja su propio estado.
    // Para consistencia con tus otros controladores, si usan static, mantenlo.
    public static WristPosition currentStatus = WristPosition.COLLECT;
    // Si no es static, cada instancia del controlador tendrá su propio previousStatus
    private WristPosition previousStatus = null;

    private Servo intakeWristServo = null;

    // --- POSICIONES DEL SERVO (¡AJUSTA ESTOS VALORES!) ---
    // Estos valores son entre 0.0 y 1.0
    public static double COLLECT_POSITION_SERVO = 0;  // Ejemplo: Centro
    public static double GRAB_POSITION_SERVO = 0.5;  // Ejemplo: Hacia un extremo
    public static double SCORE_MID_POSITION_SERVO = 1;  // <<<--- VALOR PARA LA NUEVA POSICIÓN (ejemplo)

    public IntakeWristController(RobotMap robot) {
        if (robot == null || robot.intakeWristServo == null) { // Verificar que el servo exista en RobotMap
            System.err.println("IntakeWristController Error: RobotMap o intakeWristServo es NULL.");
            // Considera cómo manejar esto. Si el servo es crítico, podrías lanzar una excepción.
            // O simplemente el controlador no funcionará.
            this.intakeWristServo = null; // Asegurar que es null si no se puede inicializar
            return;
        }
        this.intakeWristServo = robot.intakeWristServo; // Asignar el servo desde RobotMap
        this.previousStatus = null; // Forzar la primera actualización
    }

    public void update() {
        if (intakeWristServo == null) {
            return; // No hacer nada si el servo no está disponible
        }

        if (currentStatus != previousStatus) {
            previousStatus = currentStatus; // Actualizar el estado previo

            switch (currentStatus) {
                case COLLECT:
                    intakeWristServo.setPosition(COLLECT_POSITION_SERVO);
                    break;
                case GRAB:
                    intakeWristServo.setPosition(GRAB_POSITION_SERVO);
                    break;
                case SCORE_MID: // <<<--- AÑADIR CASO PARA LA NUEVA POSICIÓN
                    intakeWristServo.setPosition(SCORE_MID_POSITION_SERVO);
                    break;
            }
        }
    }

    // Métodos de ayuda (opcional)
    public WristPosition getCurrentStatus() {
        return currentStatus;
    }

    public void setStatus(WristPosition newStatus) {
        currentStatus = newStatus;
        // No necesitas llamar a update() aquí, el bucle principal del TeleOp lo hará.
    }
}