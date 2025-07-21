package TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.TeleOp.controllers.IntakeClawController;
import org.firstinspires.ftc.teamcode.TeleOp.controllers.IntakeRotationController;
import org.firstinspires.ftc.teamcode.TeleOp.controllers.IntakeWristController;
import org.firstinspires.ftc.teamcode.TeleOp.controllers.LiftController;

@Config
@TeleOp(name="teleoperado", group="Linear OpMode")
public class teleoperado extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    int velocidad =1;
    TouchSensor left_magnetic;

    private ServoController servoController;
    public static double p = 0, i = 0, d = 0;
    public static double f = 0;
    public static int target = 0;

    private static double power;
    private final double ticks_in_degree = 288 / 360.0;
    private static double div = 1;
    public static double liftpower = 0;


    public void setMotorRunningMode(DcMotor leftFrontDrive, DcMotor leftBackDrive, DcMotor rightFrontDrive,
                                    DcMotor rightBackDrive, DcMotor.RunMode runningMode) {
        leftFrontDrive.setMode(runningMode);
        rightFrontDrive.setMode(runningMode);
        leftBackDrive.setMode(runningMode);
        rightBackDrive.setMode(runningMode);
    }

    public void setMotorZeroPowerBehaviour(DcMotor leftFrontDrive, DcMotor leftBackDrive, DcMotor rightFrontDrive,
                                           DcMotor rightBackDrive, DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        leftFrontDrive.setZeroPowerBehavior(zeroPowerBehavior);
        rightFrontDrive.setZeroPowerBehavior(zeroPowerBehavior);
        leftBackDrive.setZeroPowerBehavior(zeroPowerBehavior);
        rightBackDrive.setZeroPowerBehavior(zeroPowerBehavior);
    }

    @Override
    public void runOpMode() {

        RobotMap robot = new RobotMap(hardwareMap);
        //DrawerController drawerController = new DrawerController(robot);

        IntakeClawController lowClawController = new IntakeClawController(robot);
        IntakeRotationController intakeRotationController = new IntakeRotationController(robot);
        LiftController liftController = new LiftController(robot);
        IntakeWristController intakeWristController = new IntakeWristController(robot);


        boolean liftActive = false; // Variable global


        lowClawController.update();
        intakeRotationController.update();

        liftController.update();
        intakeWristController.update();


        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftfront");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftback");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightfront");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightback");

        left_magnetic = hardwareMap.get(TouchSensor.class, "left_magnetic");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        //overclocking chassis motors
        MotorConfigurationType mct1, mct2, mct3, mct4, mct5;
        mct1 = rightBackDrive.getMotorType().clone();
        mct1.setAchieveableMaxRPMFraction(1.0);
        rightBackDrive.setMotorType(mct1);

        mct2 = rightFrontDrive.getMotorType().clone();
        mct2.setAchieveableMaxRPMFraction(1.0);
        rightFrontDrive.setMotorType(mct2);

        mct3 = leftFrontDrive.getMotorType().clone();
        mct3.setAchieveableMaxRPMFraction(1.0);
        leftFrontDrive.setMotorType(mct3);

        mct4 = leftBackDrive.getMotorType().clone();
        mct4.setAchieveableMaxRPMFraction(1.0);
        leftBackDrive.setMotorType(mct4);

        mct5 = leftBackDrive.getMotorType().clone();
        mct5.setAchieveableMaxRPMFraction(1.0);


        setMotorRunningMode(leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive,
                DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        setMotorZeroPowerBehaviour(leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive,
                DcMotor.ZeroPowerBehavior.BRAKE);


        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // see why you need previousGamepad1 & 2 on gm0
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        waitForStart();
        runtime.reset();

        LiftController.currentStatus = LiftController.liftStatus.INIT;

        // run until the end of the match (driver presses STOP)
        double max;
        while (opModeIsActive()) {

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);



            //EXTENDER DRAWERS Y CERRAR
            if (currentGamepad2.square && !previousGamepad2.square) {



            {

        }
        //TRANSFER SAMPLE BASKET
        if (currentGamepad2.circle && !previousGamepad2.circle) {
            {


            }
            //SCORE SAMPLE BASKET

            //VUELVE A INICIAR


            //RECOGER EL SAMPLE BASKET
        }}}}
                }

