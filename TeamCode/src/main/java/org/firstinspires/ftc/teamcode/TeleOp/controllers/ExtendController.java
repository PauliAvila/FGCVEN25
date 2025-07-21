package org.firstinspires.ftc.teamcode.TeleOp.controllers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.TeleOp.RobotMap;

public class ExtendController {

    public enum ExtendPosition {
        CERRADO,    // Retracted
        MEDIO,      // Mid extension
        ABIERTO,    // Fully extended
        APAGADO     // STOP_AND_RESET_ENCODER
    }

    public static final int RETRACTED_TICKS = 0;
    public static final int MID_EXTENDED_TICKS = 500;
    public static final int FULLY_EXTENDED_TICKS = 1615;

    public static final double EXTEND_POWER = 0.85;

    private RobotMap robot;
    private DcMotorEx leftExtensionMotor;
    private DcMotorEx rightExtensionMotor;

    public static ExtendPosition currentState = ExtendPosition.CERRADO;
    private ExtendPosition previousState = null;

    public ExtendController(RobotMap robotMap) {
        this.robot = robotMap;

        try {
            leftExtensionMotor = robot.leftExtensionMotor;
            rightExtensionMotor = robot.rightExtensionMotor;

            if (leftExtensionMotor == null || rightExtensionMotor == null)
                throw new IllegalArgumentException("Motores no configurados");

            leftExtensionMotor.setDirection(DcMotor.Direction.REVERSE);
            rightExtensionMotor.setDirection(DcMotor.Direction.FORWARD);

            leftExtensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightExtensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            stopAndResetEncoders();

            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception e) {
            System.err.println("Error inicializando ExtendController: " + e.getMessage());
        }
    }

    public void update() {
        if (currentState != previousState) {
            previousState = currentState;

            switch (currentState) {
                case CERRADO:
                    setTargetPosition(RETRACTED_TICKS);
                    setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    setPower(EXTEND_POWER);
                    break;

                case MEDIO:
                    setTargetPosition(MID_EXTENDED_TICKS);
                    setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    setPower(EXTEND_POWER);
                    break;

                case ABIERTO:
                    setTargetPosition(FULLY_EXTENDED_TICKS);
                    setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    setPower(EXTEND_POWER);
                    break;

                case APAGADO:
                    stopMotors();
                    stopAndResetEncoders();
                    break;
            }
        }
    }

    // --- Utility Methods ---
    public void goToPosition(ExtendPosition position) {
        currentState = position;
    }

    private void setTargetPosition(int ticks) {
        if (leftExtensionMotor != null) leftExtensionMotor.setTargetPosition(ticks);
        if (rightExtensionMotor != null) rightExtensionMotor.setTargetPosition(ticks);
    }

    private void setMode(DcMotor.RunMode mode) {
        if (leftExtensionMotor != null) leftExtensionMotor.setMode(mode);
        if (rightExtensionMotor != null) rightExtensionMotor.setMode(mode);
    }

    private void setPower(double power) {
        if (leftExtensionMotor != null) leftExtensionMotor.setPower(Math.abs(power));
        if (rightExtensionMotor != null) rightExtensionMotor.setPower(Math.abs(power));
    }

    public void stopAndResetEncoders() {
        if (leftExtensionMotor != null) {
            leftExtensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        if (rightExtensionMotor != null) {
            rightExtensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
    public void stopMotors() {
        setPower(0);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public boolean areMotorsBusy() {
        return (leftExtensionMotor != null && leftExtensionMotor.isBusy()) ||
                (rightExtensionMotor != null && rightExtensionMotor.isBusy());
    }

    public ExtendPosition getCurrentState() {
        return currentState;
    }
}
