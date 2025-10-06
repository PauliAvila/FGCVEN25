package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

@Config
@TeleOp(name="teleoperado", group="Linear OpMode")
public class teleoperado extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime hugTimer = new ElapsedTime();


    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    public TouchSensor rightMagnetic = null;
    public TouchSensor leftMagnetic = null;
    public DcMotor Extend = null;


    private double distancerope;


    @Override
    public void runOpMode() {
        RobotMap robot = new RobotMap(hardwareMap);

        rightMagnetic = robot.rightMagnetic;
        leftMagnetic = robot.leftMagnetic;
        Extend = robot.Extend;


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
        ExtendController.currentStatus = ExtendController.liftStatus.FREE;
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
            double extendDistance = ExtendController.Extend.getCurrentPosition();



            //EXTEND
            if (currentGamepad2.cross && !previousGamepad2.cross) {
                if (ExtendController.currentStatus == ExtendController.liftStatus.POWEROFF) {
                    HugController.currentStatus = HugController.hugStatus.INIT;
                    ExtendController.currentStatus = ExtendController.liftStatus.COLLECT;
                    hugTimer.reset();
                }
                else if (ExtendController.currentStatus == ExtendController.liftStatus.COLLECT) {
                    ExtendController.currentStatus = ExtendController.liftStatus.INIT;
                    HugController.currentStatus = HugController.hugStatus.INIT;
                    hugTimer.reset();

                } else {
                    ExtendController.currentStatus = ExtendController.liftStatus.POWEROFF;
                }
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
                } else if (FunnelController.currentStatus == FunnelController.FunnelStatus.HIGH){
                    FunnelController.currentStatus = FunnelController.FunnelStatus.MEDIUM;
                } else if (FunnelController.currentStatus == FunnelController.FunnelStatus.MEDIUM){
                    FunnelController.currentStatus = FunnelController.FunnelStatus.INIT;
                }
            }


            //HUG
           if (currentGamepad2.touchpad && !previousGamepad2.touchpad) {
                if (HugController.currentStatus == HugController.hugStatus.CLOSED) {
                    HugController.currentStatus = HugController.hugStatus.STRAIGHT;
                } else if (HugController.currentStatus == HugController.hugStatus.STRAIGHT){
                    HugController.currentStatus = HugController.hugStatus.HUG;
                } else {
                    HugController.currentStatus = HugController.hugStatus.INIT;
                    hugTimer.reset();

                }
            }

           // CERRADA DEL HUG
            if (HugController.currentStatus == HugController.hugStatus.INIT && hugTimer.seconds() > 3) {
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
            telemetry.addData("rightExtend speed", ExtendController.Extend.getVelocity());
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
            telemetry.addData("hugtimer", hugTimer.seconds());
            telemetry.addData("Hug Angle Left", hugController.hugleftservo.getPosition());
            telemetry.addData("Hug Angle Right", hugController.hugrightservo.getPosition());
            telemetry.addData("extendDistance", ExtendController.Extend.getCurrentPosition());
            telemetry.addData("Extend Status", ExtendController.currentStatus);


            telemetry.update();
        }
    }
}



