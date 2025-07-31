package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotMap {


    public DcMotorEx leftExtend;
    public DcMotorEx rightExtend;
    public DcMotorEx intake;
    public DcMotorEx hanging;


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
        hanging.setDirection(DcMotor.Direction.FORWARD);
        hanging.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hanging.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hanging.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




    }
}