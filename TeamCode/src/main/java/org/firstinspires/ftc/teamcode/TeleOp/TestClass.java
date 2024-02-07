package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
@TeleOp
public class TestClass extends LinearOpMode {
    private Blinker control_Hub;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private Gyroscope imu;
    private CRServo claw;
    private CRServo wrist;
    private DcMotor armmotorTop;
    private DcMotor armmotorBottom;
    private CRServo leftScoop;
    private CRServo rightScoop;
    private DistanceSensor dsensorFront;
    private DistanceSensor dsensorRight;
    private DistanceSensor dsensorLeft;
    private CRServo rightWheel;
    private CRServo leftWheel;
    private CRServo droneLauncher;
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        initHardware();
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            telemetry.addData("arm bottom", armmotorBottom.getCurrentPosition());
            telemetry.addData("arm top", armmotorTop.getCurrentPosition());
            telemetry.update();
            if (gamepad1.b){
                //spins backwards
                rightWheel.setPower(1);
                leftWheel.setPower(-1);
            }
            if (gamepad1.x){
                rightWheel.setPower(0);
                leftWheel.setPower(0);
            }
            double y = -gamepad1.left_stick_y * .75; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1 * .75; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x *.75;
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            if (gamepad2.back){
                droneLauncher.setPower(.5);
                sleep(500);
                droneLauncher.setPower(-.5);
                sleep(500);
                droneLauncher.setPower(0);
            }
            else{
                droneLauncher.setPower(0);
            }

            if (gamepad2.a){
                //scoop down
                //KEEP LEFT SCOOP POSITIVE AND RIGHT NEGATIVE
                //**DO NOT CHANGE
                leftScoop.setPower(.2);
                rightScoop.setPower(-.2);
            }

            if (gamepad2.y){
                //KEEP RIGHT SCOOP POSITIVE AND LEFT NEGATIVE
                //**DO NOT CHANGE
                //scoop up
                leftScoop.setPower(-.2);
                rightScoop.setPower(.2);
            }
            if (gamepad2.b){
                //grab open
                claw.setPower(1);
            }
            if (gamepad2.x){
                claw.setPower(-1);
                //close
            }

            //move arm back
            if (gamepad2.left_trigger > 0){
                armmotorTop.setPower(-.3 * gamepad2.left_trigger);
                armmotorBottom.setPower(-.6 * gamepad2.left_trigger);
                telemetry.addData("position bottom", armmotorBottom.getCurrentPosition());
                telemetry.addData("position top", armmotorBottom.getCurrentPosition());
                telemetry.update();
            }
            else if (gamepad2.right_trigger > 0){
                armmotorTop.setPower(.3 * gamepad2.right_trigger);
                //forward
                armmotorBottom.setPower(.6 * gamepad2.right_trigger);
                telemetry.addData("position bottom", armmotorBottom.getCurrentPosition());
                telemetry.addData("position top", armmotorBottom.getCurrentPosition());
                telemetry.update();

            }
            else{
                armmotorTop.setPower(0);
                armmotorBottom.setPower(0);
            }
            if (gamepad2.left_bumper){
                //back
                wrist.setPower(1);
            }
            if (gamepad2.right_bumper){
                //front
                wrist.setPower(-1);
            }
            if (gamepad1.back){
                armmotorTop.setPower(.8);
                armmotorBottom.setPower(.8);
                sleep(10000);
            }
            if (gamepad2.dpad_down){
                claw.setPower(.8);
                // set limit for claw
                wrist.setPower(-.75);
                armmotorBottom.setTargetPosition(500);
                armmotorTop.setTargetPosition(500);
                armmotorTop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armmotorBottom.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armmotorBottom.setPower(1);
                armmotorTop.setPower(.5);
                armmotorTop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armmotorBottom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armmotorBottom.setPower(.2);
                armmotorTop.setPower(.1);
                claw.setPower(-.8);
            }
            if (gamepad2.dpad_up){
                armmotorBottom.setPower(-.7);
                armmotorTop.setPower(-.35);
                sleep(550);
                armmotorBottom.setPower(0);
                armmotorTop.setPower(0);
                wrist.setPower(.75);
            }
            if (gamepad2.dpad_right){
                //claw open
                // wrist down
                //arm down
                claw.setPower(1);
                sleep(1000);
                wrist.setPower(-.75);
                sleep(500);
                armmotorBottom.setPower(.9);
                armmotorTop.setPower(.45);
                sleep(500);
                armmotorBottom.setPower(0);
                armmotorTop.setPower(0);
            }
        }
    }
    public void initHardware(){
        dsensorFront = hardwareMap.get(DistanceSensor.class, "ds");
        frontLeftMotor = hardwareMap.dcMotor.get("frontleft");
        backLeftMotor = hardwareMap.dcMotor.get("backleft");
        frontRightMotor = hardwareMap.dcMotor.get("frontright");
        backRightMotor = hardwareMap.dcMotor.get("backright");
        leftScoop = hardwareMap.crservo.get("leftScoop");
        rightScoop = hardwareMap.crservo.get("rightScoop");
        droneLauncher = hardwareMap.crservo.get("droneLauncher");
        armmotorBottom = hardwareMap.dcMotor.get("ambottom");
        armmotorTop = hardwareMap.dcMotor.get("amtop");
        wrist = hardwareMap.crservo.get("wrist");
        claw = hardwareMap.crservo.get("claw");
        rightWheel = hardwareMap.crservo.get("rightwheel");
        leftWheel = hardwareMap.crservo.get("leftwheel");
        dsensorFront = hardwareMap.get(DistanceSensor.class, "ds");
        dsensorLeft = hardwareMap.get(DistanceSensor.class, "dsl");
        dsensorRight = hardwareMap.get(DistanceSensor.class, "dsr");
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
        armmotorTop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armmotorBottom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armmotorTop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armmotorBottom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
}