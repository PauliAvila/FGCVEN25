package TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotMap {
    public DcMotorEx leftExtensionMotor = null;
    public DcMotorEx rightExtensionMotor = null;

    public DcMotorEx leftLiftExtendMotor = null;
    public DcMotorEx rightLiftExtendMotor = null;
    public DcMotorEx rightLift;
    public DcMotorEx leftLift;
    public Servo intakeWristServo = null;    // <<<--- DECLARAR EL SERVO
    public Servo intakeRotationServo = null;    // <<<--- DECLARAR EL NUEVO SERVO
    public Servo intakeClawServo = null; // <<< USA ESTE NOMBRE o el que corresponda

    public RobotMap(HardwareMap Init)
    {
        try {
            leftLift = Init.get(DcMotorEx.class, "leftLift");
            leftLift.setDirection(DcMotor.Direction.FORWARD); // ¡VERIFICA ESTA DIRECCIÓN!
            leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftLift.setTargetPosition(0);
            leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // Si aplicas potencia aquí para Lift, hazlo consistentemente
            // leftLift.setPower(LIFT_POWER);

            rightLift = Init.get(DcMotorEx.class, "rightLift");
            rightLift.setDirection(DcMotor.Direction.REVERSE); // ¡VERIFICA ESTA DIRECCIÓN!
            rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightLift.setTargetPosition(0);
            rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // rightLift.setPower(LIFT_POWER);

        } catch (Exception e) {
            System.err.println("RobotMap Error: No se pudieron inicializar motores de Lift: " + e.getMessage());
            // Considera cómo manejar esto, quizás telemetría si tienes acceso o lanzar la excepción.
        }


        // --- CONFIGURACIÓN DE NUEVOS MOTORES DE EXTENSIÓN DEL LIFT ---
        try {
            leftLiftExtendMotor = Init.get(DcMotorEx.class, "leftLiftExtend"); // Usa tu nombre de configuración
            rightLiftExtendMotor = Init.get(DcMotorEx.class, "rightLiftExtend"); // Usa tu nombre de configuración

            // 1. Establecer Dirección (¡MUY IMPORTANTE!)
            // Asegúrate de que las direcciones sean correctas para que ambos motores
            // trabajen juntos para extender/retraer. Uno podría necesitar ser REVERSE.
            // Ejemplo:
            leftLiftExtendMotor.setDirection(DcMotor.Direction.FORWARD);
            rightLiftExtendMotor.setDirection(DcMotor.Direction.REVERSE); // O ambos FORWARD si es necesario

            // 2. Comportamiento con Potencia Cero (¡IMPORTANTE!)
            leftLiftExtendMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightLiftExtendMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // 3. NO MÁS resetear encoders o establecer RUN_TO_POSITION aquí para estos motores,
            //    el LiftExtendController se encargará de ponerlos en RUN_WITHOUT_ENCODER.
            //    Simplemente inicializamos la potencia a 0.
            leftLiftExtendMotor.setPower(0);
            rightLiftExtendMotor.setPower(0);


            System.out.println("RobotMap: Motores LiftExtend inicializados para control manual (sin encoders).");

        } catch (Exception e) {
            System.err.println("RobotMap Error: No se pudieron inicializar motores LiftExtend: " + e.getMessage());
            leftLiftExtendMotor = null; // Asegurar que sean null si fallan
            rightLiftExtendMotor = null;
        }

        // --- CONFIGURACIÓN DEL SERVO DEL INTAKE WRIST --- // <<<--- NUEVA SECCIÓN
        try {
            // ¡USA EL NOMBRE EXACTO DE TU CONFIGURACIÓN DEL ROBOT CONTROLLER!
            intakeWristServo = Init.get(Servo.class, "intakeWrist");
            System.out.println("RobotMap: Servo 'intakeWristServo' inicializado.");

            // Opcional: Establecer una posición inicial por defecto desde RobotMap
            // si no quieres que IntakeWristController lo haga en su primera update().
            // Si IntakeWristController.INIT_POSITION_SERVO es la que quieres, no es necesario aquí.
            // intakeWristServo.setPosition(IntakeWristController.INIT_POSITION_SERVO);

        } catch (Exception e) {
            System.err.println("RobotMap Error: No se pudo inicializar 'intakeWristServo': " + e.getMessage());
            intakeWristServo = null; // Asegurar que sea null si falla
        }
// --- CONFIGURACIÓN DEL SERVO DE ROTACIÓN DEL INTAKE --- // <<<--- NUEVA SECCIÓN
        try {
            // ¡USA EL NOMBRE EXACTO DE TU CONFIGURACIÓN DEL ROBOT CONTROLLER PARA ESTE SERVO!
            intakeRotationServo = Init.get(Servo.class, "intakeRotationClaw");
            System.out.println("RobotMap: Servo 'intakeRotationServo' inicializado.");
        } catch (Exception e) {
            System.err.println("RobotMap Error: No se pudo inicializar 'intakeRotationServo': " + e.getMessage());
            intakeRotationServo = null; // Asegurar que sea null si falla
        }
// ...
        try {
            // <<<--- INICIALIZAR INTAKE CLAW SERVO --- >>>
            // CAMBIA "intake_claw_servo_config_name" AL NOMBRE EXACTO EN TU CONFIGURACIÓN
            intakeClawServo = Init.get(Servo.class, "intakeClaw");
            System.out.println("RobotMap: Servo 'intakeClawServo' inicializado.");
        } catch (Exception e) {
            System.err.println("RobotMap Error: No se pudo inicializar 'intakeClawServo': " + e.getMessage());
        }

       //GLOBAL

        try {
            leftExtensionMotor = Init.get(DcMotorEx.class, "leftExtend"); // Usa tus nombres de config
            rightExtensionMotor = Init.get(DcMotorEx.class, "rightExtend"); // Usa tus nombres de config

            // La dirección y el reseteo de encoders se manejarán en ExtendController
            // Pero puedes establecer el ZeroPowerBehavior aquí si quieres
            leftExtensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightExtensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // NO pongas RUN_TO_POSITION o STOP_AND_RESET_ENCODER aquí si el controlador lo hace.
            // Solo asegura que los motores estén configurados para usar encoders.
            leftExtensionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightExtensionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        } catch (Exception e) {
            System.err.println("Error en RobotMap al inicializar motores de extensión: " + e.getMessage());
            // Marcar que hubo un error o lanzar una excepción puede ser útil
        }
    }
}