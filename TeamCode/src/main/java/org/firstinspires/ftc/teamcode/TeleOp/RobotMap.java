package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotMap {
    public DcMotorEx leftExtensionMotor = null;
    public DcMotorEx rightExtensionMotor = null;



    public RobotMap(HardwareMap Init)
    {

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