package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.TeleOp.controllers.ExtendController;
import org.firstinspires.ftc.teamcode.TeleOp.controllers.FunnelController;
import org.firstinspires.ftc.teamcode.TeleOp.controllers.IntakeController;
import org.firstinspires.ftc.teamcode.TeleOp.controllers.HangingController;
import org.firstinspires.ftc.teamcode.TeleOp.controllers.MultiSensorController;
import org.firstinspires.ftc.teamcode.TeleOp.controllers.RampController;
import org.firstinspires.ftc.teamcode.TeleOp.controllers.MultiSensorController;

@Config
@TeleOp(name="teleoperado", group="Linear OpMode")
public class teleoperado extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private double distancerope;

    @Override
    public void runOpMode() {
        RobotMap robot = new RobotMap(hardwareMap);

        //DRIVETRAIN
        leftDrive=hardwareMap.get(DcMotorEx.class,"leftDrive");
        leftDrive.setDirection(DcMotor.Direction.REVERSE);

        rightDrive=hardwareMap.get(DcMotorEx.class,"rightDrive");
        rightDrive.setDirection(DcMotor.Direction.FORWARD);


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

        MultiSensorController multiSensorController;
        multiSensorController = new MultiSensorController(robot);
        multiSensorController.update();

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



        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            distancerope = MultiSensorController.distance.getDistance(DistanceUnit.CM);

            if (currentGamepad2.cross && !previousGamepad2.cross) {
                if (ExtendController.currentStatus == ExtendController.liftStatus.INIT) {
                    ExtendController.currentStatus = ExtendController.liftStatus.COLLECT;

                } else if (ExtendController.currentStatus == ExtendController.liftStatus.COLLECT) {
                    ExtendController.currentStatus = ExtendController.liftStatus.INIT;


                } else if (ExtendController.currentStatus == ExtendController.liftStatus.POWEROFF) {
                    ExtendController.currentStatus = ExtendController.liftStatus.COLLECT;
                }
            }


            //INTAKE
            if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) {
                if (IntakeController.currentStatus == IntakeController.intakeStatus.POWEROFF) {
                    IntakeController.currentStatus = IntakeController.intakeStatus.FORWARD;
                }
                else {
                    IntakeController.currentStatus = IntakeController.intakeStatus.POWEROFF;
                }
            }

            if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
                if (IntakeController.currentStatus == IntakeController.intakeStatus.POWEROFF) {
                    IntakeController.currentStatus = IntakeController.intakeStatus.REVERSE;
                }
                else {
                    IntakeController.currentStatus = IntakeController.intakeStatus.POWEROFF;
                }
            }


            //HANGING
            if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {
                if (HangingController.currentStatus == HangingController.hangingStatus.POWEROFF) {
                    HangingController.currentStatus = HangingController.hangingStatus.HANG;
                }
                else {
                    HangingController.currentStatus = HangingController.hangingStatus.POWEROFF;
                }
            }

            if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {
                if (HangingController.currentStatus == HangingController.hangingStatus.POWEROFF) {
                    HangingController.currentStatus = HangingController.hangingStatus.UNHANG;
                }
                else {
                    HangingController.currentStatus = HangingController.hangingStatus.POWEROFF;
                }
            }

            //SENSOR
            if (currentGamepad2.circle && !previousGamepad2.circle) {
                if (MultiSensorController.currentStatus == MultiSensorController.sensorStatus.OFF) {
                    MultiSensorController.currentStatus = MultiSensorController.sensorStatus.ON;

                } else {
                    MultiSensorController.currentStatus = MultiSensorController.sensorStatus.OFF;
                }
            }

            if (distancerope < 5){
                if (MultiSensorController.currentStatus == MultiSensorController.sensorStatus.ON){
                    HangingController.currentStatus = HangingController.hangingStatus.HANG;
                }
            }

            //RAMP
            if (currentGamepad2.triangle && !previousGamepad2.triangle) {
                if (RampController.currentStatus == RampController.RampStatus.INIT) {
                    RampController.currentStatus = RampController.RampStatus.HIGH;
                }
                else {
                    RampController.currentStatus = RampController.RampStatus.INIT;
                }
            }


            //FUNNEL
            if (currentGamepad2.square && !previousGamepad2.square) {
                if (FunnelController.currentStatus == FunnelController.FunnelStatus.INIT) {
                    FunnelController.currentStatus = FunnelController.FunnelStatus.HIGH;
                }
                else {
                    FunnelController.currentStatus = FunnelController.FunnelStatus.INIT;
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

            // --- NORMALIZACIÓN/RECORTE DE POTENCIA ---
            // Asegura que la potencia no exceda +/- 1.0
            // Opción 1: Normalización si alguna potencia excede 1.0
            double max = Math.max(Math.abs(leftMotorPower), Math.abs(rightMotorPower));
            if (max > 1.0) {
                leftMotorPower /= max;
                rightMotorPower /= max;
            }

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
            multiSensorController.update();


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("leftExtend", ExtendController.rightExtend.getCurrentPosition());
            telemetry.addData("rightExtend", ExtendController.leftExtend.getCurrentPosition());
            telemetry.addData("rightExtend speed", ExtendController.rightExtend.getVelocity());
            telemetry.addData("leftExtend speed", ExtendController.leftExtend.getVelocity());


            telemetry.addData("Intake Status", IntakeController.currentStatus);


            telemetry.addData("Hanging Status", HangingController.currentStatus);
            telemetry.addData("velocidad core", HangingController.hangingCore.getVelocity());
            telemetry.addData("velocidad ultraplanetary", HangingController.hanging.getVelocity());


            telemetry.addData("Ramp Status", RampController.currentStatus);
            telemetry.addData("Funnel Status", FunnelController.currentStatus);


            telemetry.addData("Distance Status", MultiSensorController.currentStatus);
            telemetry.addData("Distance", MultiSensorController.distancerope);
            telemetry.addData("LeftColor Distance", MultiSensorController.leftProximity);
            telemetry.addData("RightColor Distance", MultiSensorController.rightProximity);

            telemetry.addData("LeftBlue Value", MultiSensorController.leftBlue);
            telemetry.addData("LeftGreen Value", MultiSensorController.leftGreen);
            telemetry.addData("LeftRed Value", MultiSensorController.leftRed);

            telemetry.addData("RightBlue Value", MultiSensorController.rightBlue);
            telemetry.addData("RightGreen Value", MultiSensorController.rightGreen);
            telemetry.addData("RightRed Value", MultiSensorController.rightRed);


            telemetry.update();
        }
    }
}