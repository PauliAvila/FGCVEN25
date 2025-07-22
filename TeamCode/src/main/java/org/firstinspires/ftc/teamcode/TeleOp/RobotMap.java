package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotMap {


    public DcMotorEx leftExtend;
    public DcMotorEx rightExtend;


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



    }
}