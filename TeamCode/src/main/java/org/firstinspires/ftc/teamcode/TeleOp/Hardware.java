package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Hardware {
    HardwareMap hardwareMap = null;


    public Hardware(){
        initialize(hardwareMap);
    }


    private void initialize(HardwareMap hwMap){
        hardwareMap = hwMap;


        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontleft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backleft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontright");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backright");
        CRServo leftScoop = hardwareMap.crservo.get("leftScoop");
        CRServo rightScoop = hardwareMap.crservo.get("rightScoop");
        CRServo droneLauncher = hardwareMap.crservo.get("droneLauncher");
        DcMotor armmotorBottom = hardwareMap.dcMotor.get("ambottom");
        DcMotor armmotorTop = hardwareMap.dcMotor.get("amtop");
        CRServo wrist = hardwareMap.crservo.get("wrist");
        CRServo claw = hardwareMap.crservo.get("claw");
        CRServo rightWheel = hardwareMap.crservo.get("rightwheel");
        CRServo leftWheel = hardwareMap.crservo.get("leftwheel");

        //reverses
        droneLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //gearboxes require 3 of the motors to be reversed
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



    }



}
