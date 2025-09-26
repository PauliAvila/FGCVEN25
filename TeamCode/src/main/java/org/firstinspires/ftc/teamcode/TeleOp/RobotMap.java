package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;

public class RobotMap {

    public DcMotorEx Accelerator;
    public DcMotorEx rightExtend;

    public DcMotorEx intake;

    public DcMotorEx hanging;
    public DcMotorEx hangingCore;


    public Servo hugleftservo;
    public Servo hugrightservo;
    public Servo rightFunnel;
    public Servo leftFunnel;

    public Servo leftramp;
    public Servo rightramp;

    public DistanceSensor distance;


    public RobotMap(HardwareMap Init)
    {


        Accelerator=Init.get(DcMotorEx.class,"Accelerator");
        Accelerator.setDirection(DcMotor.Direction.REVERSE);
        Accelerator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Accelerator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Accelerator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightExtend=Init.get(DcMotorEx.class,"rightExtend");
        rightExtend.setDirection(DcMotor.Direction.FORWARD);
        rightExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //INTAKE
        intake=Init.get(DcMotorEx.class,"intake");
        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //HANGING
        hanging=Init.get(DcMotorEx.class,"hanging");
        hanging.setDirection(DcMotor.Direction.REVERSE);
        hanging.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hanging.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hanging.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        hangingCore=Init.get(DcMotorEx.class, "hangingCore");
        hangingCore.setDirection(DcMotorEx.Direction.FORWARD);
        hangingCore.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        hangingCore.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        hangingCore.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);



        //FUNNEL
        rightFunnel=Init.get(Servo.class,"rightFunnel");
        leftFunnel=Init.get(Servo.class,"leftFunnel");

        //DISTANCE SENSOR
        distance=Init.get(DistanceSensor.class, "distance");

        hugleftservo=Init.get(Servo.class,"hugleftservo");
        hugrightservo=Init.get(Servo.class,"hugrightservo");

        //RAMP
        rightramp=Init.get(Servo.class,"rightramp");
        leftramp=Init.get(Servo.class,"leftramp");



    }
}