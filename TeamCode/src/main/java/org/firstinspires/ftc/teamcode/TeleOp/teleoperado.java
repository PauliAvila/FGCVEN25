package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.TeleOp.controllers.AcceleratorController;
import org.firstinspires.ftc.teamcode.TeleOp.controllers.DistanceSensorController;
import org.firstinspires.ftc.teamcode.TeleOp.controllers.ExtendController;
import org.firstinspires.ftc.teamcode.TeleOp.controllers.FunnelController;
import org.firstinspires.ftc.teamcode.TeleOp.controllers.HangingController;
import org.firstinspires.ftc.teamcode.TeleOp.controllers.HugController;
import org.firstinspires.ftc.teamcode.TeleOp.controllers.IntakeController;
import org.firstinspires.ftc.teamcode.TeleOp.controllers.RampController;
import com.qualcomm.robotcore.hardware.ColorSensor;
import android.graphics.Color;


@Config
@TeleOp(name="teleoperado", group="Linear OpMode")
public class teleoperado extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime hugTimer = new ElapsedTime();
    private final ElapsedTime AccelerateTimer = new ElapsedTime();
    private final ElapsedTime RumbleTimer = new ElapsedTime();


    ColorSensor colorSensor;

    // ---- Rangos de Hue configurables ----
    final float[] RED_HUE_RANGE = {0f, 30f};
    final float[] ORANGE_HUE_RANGE = {34f, 70f};
    final float[] YELLOW_HUE_RANGE = {80f, 100f};
    final float[] BLUE_HUE_RANGE = {200f, 250f};

    // ---- Parámetros de saturación y brillo ----
    final float MIN_SATURATION = 0.45f;
    final float MIN_VALUE = 0.35f;


    private final ElapsedTime extendTimer = new ElapsedTime();

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    public TouchSensor rightMagnetic = null;
    public TouchSensor leftMagnetic = null;


    private double distancerope;


    @Override
    public void runOpMode() {
        RobotMap robot = new RobotMap(hardwareMap);

        rightMagnetic = robot.rightMagnetic;
        leftMagnetic = robot.leftMagnetic;


        //DRIVETRAIN
        leftDrive=hardwareMap.get(DcMotorEx.class,"leftDrive");
        leftDrive.setDirection(DcMotor.Direction.REVERSE);

        rightDrive=hardwareMap.get(DcMotorEx.class,"rightDrive");
        rightDrive.setDirection(DcMotor.Direction.FORWARD);


        //COLOR SENSOR
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        telemetry.addLine("📊 Iniciando sensor de color...");
        telemetry.addLine("Mantén el sensor frente al color que deseas calibrar.");



        //CONTROLLERS
        ExtendController extendController;
        extendController = new ExtendController(robot);
        extendController.update(0);

        IntakeController intakeController;
        intakeController = new IntakeController(robot);
        intakeController.update(0);

        HangingController hangingController;
        hangingController = new HangingController(robot);
        hangingController.update(0);

        RampController rampController;
        rampController = new RampController(robot);
        rampController.update();

        FunnelController funnelController;
        funnelController = new FunnelController(robot);
        funnelController.update();

        DistanceSensorController distanceSensorController;
        distanceSensorController = new DistanceSensorController(robot);
        distanceSensorController.update();

        AcceleratorController acceleratorController;
        acceleratorController = new AcceleratorController(robot);
        acceleratorController.update();

        HugController hugController;
        hugController = new HugController(robot);
        hugController.update();



        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // see why you need previous Gamepad1 & 2 on gm0
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();
        FunnelController.currentStatus = FunnelController.FunnelStatus.INIT;

        funnelController.update();

        waitForStart();

        runtime.reset();
        ExtendController.currentStatus = ExtendController.liftStatus.POWEROFF;
        IntakeController.currentStatus = IntakeController.intakeStatus.POWEROFF;
        HangingController.currentStatus = HangingController.hangingStatus.POWEROFF;
        FunnelController.currentStatus = FunnelController.FunnelStatus.HIGH;
        AcceleratorController.currentStatus = AcceleratorController.acceleratorStatus.OFF;
        HugController.currentStatus = HugController.hugStatus.CLOSED;




        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            distancerope = DistanceSensorController.distance.getDistance(DistanceUnit.CM);

            float[] hsv = new float[3];
            Color.RGBToHSV(
                    colorSensor.red() * 8,
                    colorSensor.green() * 8,
                    colorSensor.blue() * 8,
                    hsv
            );

            float hue = hsv[0];
            float sat = hsv[1];
            float val = hsv[2];

            // ---- Mostrar datos en tiempo real ----
            telemetry.addLine("🎨 VALORES EN VIVO");
            telemetry.addData("Hue", "%.1f°", hue);
            telemetry.addData("Saturación", "%.2f", sat);
            telemetry.addData("Valor (brillo)", "%.2f", val);
            telemetry.addLine();

            // ---- Mostrar rangos configurados ----
            telemetry.addLine("🎯 RANGOS CONFIGURADOS:");
            telemetry.addData("Rojo", "[%.0f° - %.0f°]", RED_HUE_RANGE[0], RED_HUE_RANGE[1]);
            telemetry.addData("Naranja", "[%.0f° - %.0f°]", ORANGE_HUE_RANGE[0], ORANGE_HUE_RANGE[1]);
            telemetry.addData("Amarillo", "[%.0f° - %.0f°]", YELLOW_HUE_RANGE[0], YELLOW_HUE_RANGE[1]);
            telemetry.addData("Azul", "[%.0f° - %.0f°]", BLUE_HUE_RANGE[0], BLUE_HUE_RANGE[1]);
            telemetry.addData("Mín. Saturación", "%.2f", MIN_SATURATION);
            telemetry.addData("Mín. Valor", "%.2f", MIN_VALUE);
            telemetry.addLine();

            // ---- Detectar color actual ----
            if (isRed(hsv)) {
                telemetry.addLine("🔴 Detectado: ROJO");
            } else if (isYellow(hsv)) {
                telemetry.addLine("🟨 Detectado: AMARILLO");
            } else if (isBlue(hsv)) {
                telemetry.addLine("🔵 Detectado: AZUL");
            } else {
                telemetry.addLine("⚪ Ningún color reconocido");
            }


            if (currentGamepad1.circle && !previousGamepad1.circle){
                AccelerateTimer.reset();

            }
            if (AccelerateTimer.seconds() > 20){

                if (RumbleTimer.seconds() > 3){
                gamepad1.rumble(500);
                gamepad2.rumble(500);
                RumbleTimer.reset();
                }
            }

            //EXTEND
            if (currentGamepad2.cross && !previousGamepad2.cross) {
                if (ExtendController.currentStatus == ExtendController.liftStatus.POWEROFF) {
                    HugController.currentStatus = HugController.hugStatus.INIT;
                    ExtendController.currentStatus = ExtendController.liftStatus.COLLECT;
                    hugTimer.reset();
                } else {
                    ExtendController.currentStatus = ExtendController.liftStatus.INIT;
                    HugController.currentStatus = HugController.hugStatus.INIT;
                    hugTimer.reset();
                }
            }

            if (rightMagnetic.isPressed() && leftMagnetic.isPressed() ) {
                if (ExtendController.currentStatus == ExtendController.liftStatus.INIT) {
                    ExtendController.currentStatus = ExtendController.liftStatus.POWEROFF;
                }
            }

            if (ExtendController.currentStatus == ExtendController.liftStatus.COLLECT && ExtendController.extend.getCurrentPosition() > 2400){
                ExtendController.currentStatus = ExtendController.liftStatus.POWEROFFEXTEND;
            }

            boolean isOrange = (hue >= ORANGE_HUE_RANGE[0] && hue <= ORANGE_HUE_RANGE[1])
                    && (sat >= MIN_SATURATION)
                    && (val >= MIN_VALUE);

            if (isOrange) {
                telemetry.addLine("🟧 NARANJA DETECTADO — Acelerador activado automáticamente");
                AcceleratorController.currentStatus = AcceleratorController.acceleratorStatus.ACCELERATE;
            }


            //ACCELERATOR
            if (currentGamepad2.right_stick_button && !previousGamepad2.right_stick_button) {
                if (AcceleratorController.currentStatus == AcceleratorController.acceleratorStatus.OFF) {
                    AcceleratorController.currentStatus = AcceleratorController.acceleratorStatus.ACCELERATE;
                } else {
                    AcceleratorController.currentStatus = AcceleratorController.acceleratorStatus.OFF;
                }
            }

            if (currentGamepad2.left_stick_button && !previousGamepad2.left_stick_button) {
                if (AcceleratorController.currentStatus == AcceleratorController.acceleratorStatus.OFF) {
                    AcceleratorController.currentStatus = AcceleratorController.acceleratorStatus.DESACCELERATE;
                } else {
                    AcceleratorController.currentStatus = AcceleratorController.acceleratorStatus.OFF;
                }
            }

            //INTAKE
            if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) {
                if (IntakeController.currentStatus == IntakeController.intakeStatus.POWEROFF) {
                    IntakeController.currentStatus = IntakeController.intakeStatus.FORWARD;
                    AcceleratorController.currentStatus = AcceleratorController.acceleratorStatus.ACCELERATE;
                } else {
                    IntakeController.currentStatus = IntakeController.intakeStatus.POWEROFF;
                    AcceleratorController.currentStatus = AcceleratorController.acceleratorStatus.OFF;

                }
            }

            if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
                if (IntakeController.currentStatus == IntakeController.intakeStatus.POWEROFF) {
                    IntakeController.currentStatus = IntakeController.intakeStatus.REVERSE;
                    AcceleratorController.currentStatus = AcceleratorController.acceleratorStatus.DESACCELERATE;

                } else {
                    IntakeController.currentStatus = IntakeController.intakeStatus.POWEROFF;
                    AcceleratorController.currentStatus = AcceleratorController.acceleratorStatus.OFF;
                }
            }


            //HANGING
            if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {
                if (HangingController.currentStatus == HangingController.hangingStatus.POWEROFF) {
                    HangingController.currentStatus = HangingController.hangingStatus.HANG;
                } else {
                    HangingController.currentStatus = HangingController.hangingStatus.POWEROFF;
                }
            }

            if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {
                if (HangingController.currentStatus == HangingController.hangingStatus.POWEROFF) {
                    HangingController.currentStatus = HangingController.hangingStatus.UNHANG;
                } else {
                    HangingController.currentStatus = HangingController.hangingStatus.POWEROFF;
                }
            }

            //SENSOR
            if (currentGamepad2.circle && !previousGamepad2.circle) {
                if (DistanceSensorController.currentStatus == DistanceSensorController.distanceSensorStatus.OFF) {
                    DistanceSensorController.currentStatus = DistanceSensorController.distanceSensorStatus.ON;

                } else {
                    DistanceSensorController.currentStatus = DistanceSensorController.distanceSensorStatus.OFF;
                }
            }

            if (distancerope < 15) {
                if (DistanceSensorController.currentStatus == DistanceSensorController.distanceSensorStatus.ON) {
                    HangingController.currentStatus = HangingController.hangingStatus.HANG;

                }
            }

            //RAMP
            if (currentGamepad2.triangle && !previousGamepad2.triangle) {
                if (RampController.currentStatus == RampController.RampStatus.INIT) {
                    RampController.currentStatus = RampController.RampStatus.HIGH;
                } else {
                    RampController.currentStatus = RampController.RampStatus.INIT;
                }
            }


            //FUNNEL
            if (currentGamepad2.square && !previousGamepad2.square) {
                if (FunnelController.currentStatus == FunnelController.FunnelStatus.INIT) {
                    FunnelController.currentStatus = FunnelController.FunnelStatus.HIGH;
                } else if (FunnelController.currentStatus == FunnelController.FunnelStatus.HIGH) {
                    FunnelController.currentStatus = FunnelController.FunnelStatus.MEDIUM;
                } else if (FunnelController.currentStatus == FunnelController.FunnelStatus.MEDIUM) {
                    FunnelController.currentStatus = FunnelController.FunnelStatus.INIT;
                }
            }


            //HUG
            if (currentGamepad2.touchpad && !previousGamepad2.touchpad) {
                if (HugController.currentStatus == HugController.hugStatus.CLOSED) {
                    HugController.currentStatus = HugController.hugStatus.STRAIGHT;
                } else if (HugController.currentStatus == HugController.hugStatus.STRAIGHT) {
                    HugController.currentStatus = HugController.hugStatus.HUG;
                } else {
                    HugController.currentStatus = HugController.hugStatus.INIT;
                    hugTimer.reset();

                }
            }

            // CERRADA DEL HUG
            if (ExtendController.currentStatus == ExtendController.liftStatus.POWEROFF && HugController.currentStatus == HugController.hugStatus.INIT && hugTimer.seconds() > 3.3) {
                HugController.currentStatus = HugController.hugStatus.CLOSED;

            }

            //HUG EN GAMEPAD 1
            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                if (HugController.currentStatus == HugController.hugStatus.STRAIGHT) {
                    HugController.currentStatus = HugController.hugStatus.LEFT_FLOW;
                } else {
                    HugController.currentStatus = HugController.hugStatus.STRAIGHT;
                }
            }
            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
                if (HugController.currentStatus == HugController.hugStatus.STRAIGHT) {
                    HugController.currentStatus = HugController.hugStatus.RIGHT_FLOW;
                } else {
                    HugController.currentStatus = HugController.hugStatus.STRAIGHT;
                }
            }


            //DRIVETRAIN
            //Uses left joystick to go forward & strafe, and right joystick to rotate.
            double drivePower = gamepad1.left_stick_y;

            // Joystick derecho X para girar
            double turnPower = -gamepad1.right_stick_x;

            // --- CÁLCULO DE POTENCIA PARA CADA MOTOR ---
            double leftMotorPower = drivePower + turnPower;
            double rightMotorPower = drivePower - turnPower;

            // --- APLICAR POTENCIA A LOS MOTORES ---
            if (leftDrive != null) {
                leftDrive.setPower(leftMotorPower);
            }
            if (rightDrive != null) {
                rightDrive.setPower(rightMotorPower);
            }

            extendController.update(0);
            intakeController.update(0);
            hangingController.update(0);
            funnelController.update();
            rampController.update();
            distanceSensorController.update();
            acceleratorController.update();
            hugController.update();


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("rightExtend speed", ExtendController.extend.getVelocity());
            telemetry.addData("Intake Status", IntakeController.currentStatus);
            telemetry.addData("Hanging Status", HangingController.currentStatus);
            telemetry.addData("velocidad core", HangingController.hangingCore.getVelocity());
            telemetry.addData("velocidad ultraplanetary", HangingController.hanging.getVelocity());
            telemetry.addData("Ramp Status", RampController.currentStatus);
            telemetry.addData("Funnel Status", FunnelController.currentStatus);
            telemetry.addData("Distance Status", DistanceSensorController.currentStatus);
            telemetry.addData("Distance", DistanceSensorController.distance.getDistance(DistanceUnit.CM));
            telemetry.addData("Accelerator Status", AcceleratorController.currentStatus);
            telemetry.addData("Hug Status", HugController.currentStatus);
            telemetry.addData("Hugtimer", hugTimer.seconds());
            telemetry.addData("Hug Angle Left", hugController.hugleftservo.getPosition());
            telemetry.addData("Hug Angle Right", hugController.hugrightservo.getPosition());
            telemetry.addData("Extend Status", ExtendController.currentStatus);
            telemetry.addData("Magnetic Right Status",rightMagnetic.isPressed());
            telemetry.addData("Magnetic Left Status",leftMagnetic.isPressed());
            telemetry.addData("Posicion extend", ExtendController.extend.getCurrentPosition());
            telemetry.addData("Acceleration Timer", AccelerateTimer.seconds());
            telemetry.addData("Rumble Timer", RumbleTimer.seconds());

            telemetry.update();
        }
    }

    boolean isRed(float[] hsv) {
        return ((hsv[0] >= RED_HUE_RANGE[0] && hsv[0] <= RED_HUE_RANGE[1]) ||
                (hsv[0] >= 350 && hsv[0] <= 360)) &&
                hsv[1] >= MIN_SATURATION && hsv[2] >= MIN_VALUE;
    }



    boolean isYellow(float[] hsv) {
        return (hsv[0] >= YELLOW_HUE_RANGE[0] && hsv[0] <= YELLOW_HUE_RANGE[1]) &&
                hsv[1] >= MIN_SATURATION && hsv[2] >= MIN_VALUE;
    }

    boolean isBlue(float[] hsv) {
        return (hsv[0] >= BLUE_HUE_RANGE[0] && hsv[0] <= BLUE_HUE_RANGE[1]) &&
                hsv[1] >= MIN_SATURATION && hsv[2] >= MIN_VALUE;
    }
}




