package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;


public class RobotMap {

    public DcMotorEx Accelerator;
    public DcMotorEx Extend;

    public DcMotorEx intake;

    public DcMotorEx hangingright;
    public DcMotorEx hangingleft;


    public Servo hugleftservo;
    public Servo hugrightservo;
    public Servo rightFunnel;
    public Servo leftFunnel;

    public Servo leftramp;
    public Servo rightramp;


    public DistanceSensor distance;


    public RobotMap(HardwareMap Init)
    {

        //SENSORES MAGNETICOS

        //ACCELERATOR
        Accelerator=Init.get(DcMotorEx.class,"Accelerator");
        Accelerator.setDirection(DcMotor.Direction.REVERSE);
        Accelerator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Accelerator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Accelerator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //EXTENSION
        Extend =Init.get(DcMotorEx.class,"Extend");
        Extend.setDirection(DcMotor.Direction.FORWARD);
        Extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //INTAKE
        intake=Init.get(DcMotorEx.class,"intake");
        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //HANGING
        //overclocking chassis motors
        hangingright =Init.get(DcMotorEx.class,"hangingright");
        hangingright.setDirection(DcMotor.Direction.REVERSE);
        hangingright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangingright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangingright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        hangingleft =Init.get(DcMotorEx.class, "hangingleft");
        hangingleft.setDirection(DcMotorEx.Direction.REVERSE);
        hangingleft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        hangingleft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        hangingleft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        //FUNNEL
        rightFunnel=Init.get(Servo.class,"rightFunnel");
        leftFunnel=Init.get(Servo.class,"leftFunnel");

        //DISTANCE SENSOR
        distance=Init.get(DistanceSensor.class, "distance");

        //HUG
        hugleftservo=Init.get(Servo.class,"hugleftservo");
        hugrightservo=Init.get(Servo.class,"hugrightservo");

        //RAMP
        rightramp=Init.get(Servo.class,"rightramp");
        leftramp=Init.get(Servo.class,"leftramp");

    }
}