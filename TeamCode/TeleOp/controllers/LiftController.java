package TeleOp.controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.TeleOp.RobotMap;
// Se elimina: import android.webkit.WebSettings; // No es necesario para los estados del lift

@Config
public class LiftController {

    public static LiftStatus liftStatus;

    // 1. Definir el enum para los estados del elevador
    public enum LiftStatus { // Renombrado de liftStatus a LiftStatus por convención de nombres de enum
        INIT,
        MEDIUM,
        HIGH,
        POWEROFF
        // Puedes añadir más estados aquí si los necesitas (ej. MEDIUM, LOW)
    }

    // 2. Declarar las variables de estado
    //    'public static' si accedes directamente desde TeleOp como LiftController.currentStatus
    //    o solo 'public' si cambias el estado a través de un método en LiftController
    public static LiftStatus currentStatus = LiftStatus.INIT; // Estado actual del elevador
    private LiftStatus previousStatus = LiftStatus.INIT;   // Estado anterior para detectar cambios

    public DcMotorEx rightLift = null;
    public DcMotorEx leftLift = null;

    // --- POSICIONES ---
    public int init_position = 20;
    public int medium_position =435;
    public int high_position =600; // Ajusta este valor según sea necesario

    public static double VELOCITY_LIFT =600; // Ticks por segundo, ajusta según sea necesario

    public LiftController(RobotMap robot) {
        this.rightLift = robot.rightLift;
        this.leftLift = robot.leftLift;
        // previousStatus se inicializa arriba, no es necesario aquí.
    }

    public void update() {
        if (currentStatus != previousStatus) {
            previousStatus = currentStatus; // Actualizar el estado previo

            switch (currentStatus) {
                case INIT: { // Ahora INIT es parte de tu enum LiftStatus
                    if (this.rightLift.getTargetPosition() != init_position) {
                        this.rightLift.setTargetPosition(init_position);
                    }
                    if (this.leftLift.getTargetPosition() != init_position) {
                        this.leftLift.setTargetPosition(init_position);
                    }
                    // Asumimos que RobotMap ya los puso en RUN_TO_POSITION.
                    // Solo es necesario aplicar velocidad/potencia cuando el estado cambia para iniciar el movimiento.
                    this.rightLift.setVelocity(VELOCITY_LIFT);
                    this.leftLift.setVelocity(VELOCITY_LIFT);
                    break;
                }
                case MEDIUM: { // <--- NUEVO CASE
                    if (this.rightLift.getTargetPosition() != medium_position) {
                        this.rightLift.setTargetPosition(medium_position);
                    }
                    if (this.leftLift.getTargetPosition() != medium_position) {
                        this.leftLift.setTargetPosition(medium_position);
                    }
                    this.rightLift.setVelocity(VELOCITY_LIFT);
                    this.leftLift.setVelocity(VELOCITY_LIFT);
                    break;
                }
                case POWEROFF: { // Ahora POWEROFF es parte de tu enum LiftStatus
                    // Considera si realmente necesitas STOP_AND_RESET_ENCODER aquí.
                    // Si solo quieres detener y mantener la posición, usa setPower(0)
                    // y asegúrate de que ZeroPowerBehavior sea BRAKE.
                    // Si quieres que los motores estén "libres", usa RUN_WITHOUT_ENCODER y setPower(0).
                    // STOP_AND_RESET_ENCODER reiniciará la referencia del encoder a 0.
                    this.leftLift.setPower(0); // Detener primero
                    this.rightLift.setPower(0);
                    // Opcional: Cambiar modo si quieres que no mantengan la posición activamente
                    // this.leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    // this.rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    break;
                }

                case HIGH: { // 3. Corregido para usar tu estado HIGH
                    if (this.rightLift.getTargetPosition() != high_position) {
                        this.rightLift.setTargetPosition(high_position);
                    }
                    if (this.leftLift.getTargetPosition() != high_position) {
                        this.leftLift.setTargetPosition(high_position);
                    }
                    this.rightLift.setVelocity(VELOCITY_LIFT);
                    this.leftLift.setVelocity(VELOCITY_LIFT);
                    break;


                }
            }
        }
        // Consideración adicional:
        // Si el motor es interrumpido (ej. por un obstáculo) antes de llegar a la posición
        // y quieres que reintente activamente, podrías necesitar llamar a setVelocity()
        // incluso si currentStatus == previousStatus pero los motores no están en el target
        // y están 'isBusy()'. Sin embargo, para empezar, la lógica actual de
        // `if (currentStatus != previousStatus)` es un buen punto de partida.
        // El controlador PID interno del motor (cuando está en RUN_TO_POSITION y se le ha dado velocidad)
        // intentará alcanzar el target.
    }

    // Método para cambiar el estado desde fuera (ej. desde TeleOp) de forma más encapsulada
    public void goToState(LiftStatus newState) {
        currentStatus = newState;
    }

    // Método para obtener el estado actual (opcional, si currentStatus no es public static)
    public LiftStatus getCurrentState() {
        return currentStatus;
    }

    // El método update(int target) que tenías comentado puede quedarse así o eliminarse si no lo usarás.
    /*
    public void update(int target) {
        // ...
    }
    */
}