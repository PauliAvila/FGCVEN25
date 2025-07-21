package TeleOp.controllers; // O la ruta de tu paquete de controladores

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx; // Recomendado para mejor control de PID si está disponible
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.TeleOp.RobotMap; // Asumiendo que tienes RobotMap

public class ExtendController {

    public enum ExtendPosition {
       CERRADO,    // Posición totalmente retraída (cerca de 0 ticks)
       MEDIO, // Una posición intermedia
        ABIERTO // Extensión máxima
        // Añade más posiciones según necesites
    }

    // --- Constantes de Posición (en Ticks del Encoder) ---
    // ¡¡¡DEBES AJUSTAR ESTOS VALORES EXPERIMENTALMENTE!!!
    public static final int RETRACTED_TICKS = 0;        // Un pequeño offset para asegurar que no fuerce
    public static final int MID_EXTENDED_TICKS = 500;   // Ejemplo
    public static final int FULLY_EXTENDED_TICKS = 1615; // Ejemplo

    // --- Velocidad de Movimiento ---
    public static final double EXTEND_POWER = 0.85; // Ajustar según sea necesario

    private RobotMap robot;
    private DcMotorEx leftExtensionMotor;  // Usar DcMotorEx si es posible
    private DcMotorEx rightExtensionMotor;

    public static ExtendPosition currentTargetState = ExtendPosition.CERRADO; // Estado deseado actual
    private ExtendPosition previousTargetState = ExtendPosition.CERRADO; // Para detectar cambios

    public ExtendController(RobotMap robotMap) {
        this.robot = robotMap;
        try {
            // Obtener los motores desde RobotMap
            // Asegúrate que los nombres coincidan con tu configuración en RobotMap
            leftExtensionMotor = robot.leftExtensionMotor; // Asume que existen en RobotMap
            rightExtensionMotor = robot.rightExtensionMotor; // Asume que existen en RobotMap

            if (leftExtensionMotor == null || rightExtensionMotor == null) {
                throw new IllegalArgumentException("Los motores de extensión no están configurados en RobotMap.");
            }

            // --- Configuración Inicial de Motores ---
              leftExtensionMotor.setDirection(DcMotor.Direction.REVERSE); // AJUSTA ESTO
            rightExtensionMotor.setDirection(DcMotor.Direction.FORWARD); // AJUSTA ESTO

            // Poner los motores en modo de freno cuando no se les aplica potencia
            leftExtensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightExtensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Detener y resetear encoders al inicio para establecer la posición cero
            // Es crucial que el mecanismo esté en la posición física de "0 ticks"
            // cuando esto se ejecuta, o que sepas cuál es el offset.
            stopAndResetEncoders();

            // Establecer la posición inicial (opcional, pero bueno para empezar)
            setTargetPosition(RETRACTED_TICKS);
            setMode(DcMotor.RunMode.RUN_TO_POSITION);
            setPower(EXTEND_POWER); // Aplicar potencia para que se mueva a la posición inicial si es necesario

        } catch (Exception e) {
            // Manejar el error, quizás lanzar una RuntimeException o registrarlo
            System.err.println("Error inicializando ExtendController: " + e.getMessage());
            // Podrías tener una variable booleana 'isInitializedCorrectly' para verificar en el OpMode
        }
    }

    private void setMode(DcMotor.RunMode mode) {
        if (leftExtensionMotor != null) leftExtensionMotor.setMode(mode);
        if (rightExtensionMotor != null) rightExtensionMotor.setMode(mode);
    }

    private void setTargetPosition(int ticks) {
        if (leftExtensionMotor != null) leftExtensionMotor.setTargetPosition(ticks);
        if (rightExtensionMotor != null) rightExtensionMotor.setTargetPosition(ticks);
    }

    private void setPower(double power) {
        if (leftExtensionMotor != null) leftExtensionMotor.setPower(Math.abs(power)); // RUN_TO_POSITION usa la magnitud
        if (rightExtensionMotor != null) rightExtensionMotor.setPower(Math.abs(power));
    }

    public void stopAndResetEncoders() {
        if (leftExtensionMotor != null) {
            leftExtensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        if (rightExtensionMotor != null) {
            rightExtensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        // Después de resetear, es una buena práctica volver a un modo que permita movimiento,
        // o RUN_TO_POSITION si ya tienes un objetivo.
        // setMode(DcMotor.RunMode.RUN_USING_ENCODER); // o RUN_TO_POSITION si ya se va a mover
    }

    public void goToPosition(ExtendPosition position) {
        currentTargetState = position;
    }

    public void update() {
        if (leftExtensionMotor == null || rightExtensionMotor == null) {
            // System.out.println("ExtendController: Motores no inicializados, saltando update.");
            return; // No hacer nada si los motores no están listos
        }

        int targetTicks;
        switch (currentTargetState) {
            case CERRADO:
                targetTicks = RETRACTED_TICKS;
                break;
            case MEDIO:
                targetTicks = MID_EXTENDED_TICKS;
                break;
            case ABIERTO:
                targetTicks = FULLY_EXTENDED_TICKS;
                break;
            default:
                targetTicks = RETRACTED_TICKS; // Por defecto, ir a retraído
                break;
        }

        // Solo actualiza el target y el modo si el estado deseado ha cambiado
        // o si los motores no están ocupados (han llegado a su destino anterior)
        // Esto evita reenviar comandos constantemente y permite que RUN_TO_POSITION funcione.
        if (currentTargetState != previousTargetState || !areMotorsBusy()) {
            setTargetPosition(targetTicks);
            setMode(DcMotor.RunMode.RUN_TO_POSITION);
            setPower(EXTEND_POWER);
            previousTargetState = currentTargetState;
        }

        // Opcional: Podrías añadir lógica para detener los motores si están muy cerca del objetivo
        // y quieres evitar oscilaciones, o si quieres usar un PID más sofisticado.
        // Pero para empezar, RUN_TO_POSITION con una potencia constante es suficiente.
    }

    public boolean areMotorsBusy() {
        boolean busy = false;
        if (leftExtensionMotor != null) busy = busy || leftExtensionMotor.isBusy();
        if (rightExtensionMotor != null) busy = busy || rightExtensionMotor.isBusy();
        return busy;
    }

    public int getLeftMotorPosition() {
        return leftExtensionMotor != null ? leftExtensionMotor.getCurrentPosition() : 0;
    }

    public int getRightMotorPosition() {
        return rightExtensionMotor != null ? rightExtensionMotor.getCurrentPosition() : 0;
    }

    public ExtendPosition getCurrentTargetState() {
        return currentTargetState;
    }

    // Método para detener los motores inmediatamente (p.ej., al final del OpMode)
    public void stopMotors() {
        setPower(0); // Poner potencia a 0
        setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Cambiar a un modo que no intente ir a una posición
    }
}

