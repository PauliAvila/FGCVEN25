package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotMap {

    public DcMotorEx leftExtend;
    public DcMotorEx rightExtend;
    public DcMotorEx intake;
    public DcMotorEx hanging;
    public DcMotorEx hangingCore;
    public Servo leftRamp;
    public Servo rightRamp;
<<<<<<< HEAD

    public Servo rightFunnel;
    public Servo leftFunnel;





=======
    public Servo rightFunnel;
    public Servo leftFunnel;
>>>>>>> a816f384527bd430f19e7bbd6e9662f796e36b71


    public RobotMap(HardwareMap Init)
    {
        //EXTENDERS
        leftExtend=Init.get(DcMotorEx.class,"leftExtend");
        leftExtend.setDirection(DcMotor.Direction.REVERSE);
        leftExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightExtend=Init.get(DcMotorEx.class,"rightExtend");
        rightExtend.setDirection(DcMotor.Direction.FORWARD);
        rightExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake=Init.get(DcMotorEx.class,"intake");
        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        hanging=Init.get(DcMotorEx.class,"hanging");
        hanging.setDirection(DcMotor.Direction.REVERSE);
        hanging.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hanging.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hanging.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        hangingCore = Init.get(DcMotorEx.class, "hangingCore");
        hangingCore.setDirection(DcMotorEx.Direction.FORWARD);
        hangingCore.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        hangingCore.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        hangingCore.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        rightRamp=Init.get(Servo.class,"rightRamp");
        leftRamp=Init.get(Servo.class,"leftRamp");

        rightFunnel=Init.get(Servo.class,"rightFunnel");
        leftFunnel=Init.get(Servo.class,"leftFunnel");
<<<<<<< HEAD



=======
>>>>>>> a816f384527bd430f19e7bbd6e9662f796e36b71

    }
}