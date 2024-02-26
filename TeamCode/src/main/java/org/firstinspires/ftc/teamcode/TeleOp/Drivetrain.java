package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Drivetrain extends LinearOpMode {
    public DcMotor topLeftDriveMotor;
    public DcMotor bottomLeftDriveMotor;
    public DcMotor topRightDriveMotor;
    public DcMotor bottomRightDriveMotor;
    public static CRServo claw;
    public static CRServo wrist;
    public DcMotor armmotorTop;
    public DcMotor armmotorBottom;
    public CRServo leftScoop;
    public CRServo rightScoop;
    public CRServo rightWheel;
    public CRServo leftWheel;
    public CRServo droneLauncher;
    public DcMotorEx rightOdo;
    public DcMotorEx leftOdo;
    public DcMotorEx midOdo;
    public double Kp = 8.5;
    public double Ki = 0;
    public double Kd = 0;


    HardwareMap hwMap;

    public void runOpMode() {

    }


    public void init(HardwareMap ahwMap) {

        /**
         * Assigns the parent hardware map to local ArtemisHardwareMap class variable
         * **/
        hwMap = ahwMap;

        /**
         * Hardware initialized and String Names are in the Configuration File for Hardware Map
         * **/

        // Control HUb
        topLeftDriveMotor = hwMap.get(DcMotor.class, "frontleft");
        bottomLeftDriveMotor = hwMap.get(DcMotor.class, "backleft");
        topRightDriveMotor = hwMap.get(DcMotor.class, "frontright");
        bottomRightDriveMotor = hwMap.get(DcMotor.class, "backright");
        leftScoop = hwMap.crservo.get("leftScoop");
        rightScoop = hwMap.crservo.get("rightScoop");
        droneLauncher = hwMap.crservo.get("droneLauncher");
        droneLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
        armmotorBottom = hwMap.dcMotor.get("ambottom");
        armmotorTop = hwMap.dcMotor.get("amtop");
        wrist = hwMap.crservo.get("wrist");
        claw = hwMap.crservo.get("claw");
        rightWheel = hwMap.crservo.get("rightwheel");
        leftWheel = hwMap.crservo.get("leftwheel");
        rightOdo = hwMap.get(DcMotorEx.class, "rightodo");
        rightOdo.setDirection(DcMotorSimple.Direction.REVERSE);
        leftOdo = hwMap.get(DcMotorEx.class, "amtop");
        midOdo = hwMap.get(DcMotorEx.class, "middleodo");



        /**
         * Allow the 4 wheel motors to be run without encoders since we are doing a time based autonomous
         * **/
        topLeftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomLeftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        topRightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomRightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /**
         *Since we are putting the motors on different sides we need to reverse direction so that one wheel doesn't pull us backwards
         * **/

        //THIS IS THE CORRECT ORIENTATION
        topRightDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        bottomRightDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        topLeftDriveMotor.setDirection(DcMotor.Direction.REVERSE);

        /**
         * Reverses shooter motor to shoot the correct way and same with the conveyor motor
         * **/

        /**
         * We are setting the motor 0 mode power to be brake as it actively stops the robot and doesn't rely on the surface to slow down once the robot power is set to 0
         * **/
        topLeftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomLeftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topRightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomRightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        /**
         *The 4 mecanum wheel motors, intake, conveyor, and shooter motor/servo are set to 0 power to keep it from moving when the user presses the INIT button
         * **/
        topLeftDriveMotor.setPower(0);
        bottomLeftDriveMotor.setPower(0);
        topRightDriveMotor.setPower(0);
        bottomRightDriveMotor.setPower(0);

    }

    public void power(double output) {
        topLeftDriveMotor.setPower(-output);
        bottomLeftDriveMotor.setPower(-output);
        topRightDriveMotor.setPower(output);
        bottomRightDriveMotor.setPower(output);
    }

    public void moveRobot(double leftStickY, double leftStickX, double rightStickX) {
        /**
         * Wheel powers calculated using gamepad 1's inputs leftStickY, leftStickX, and rightStickX
         * **/
        double y = -leftStickY * .75; // Remember, Y stick value is reversed
        double x = leftStickX * 1.1 * .75; // Counteract imperfect strafing
        double rx = rightStickX * .75;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;
        topLeftDriveMotor.setPower(frontLeftPower);
        bottomLeftDriveMotor.setPower(backLeftPower);
        topRightDriveMotor.setPower(frontRightPower);
        bottomRightDriveMotor.setPower(backRightPower);
    }

    public void moveRobotSlow(double leftStickY, double leftStickX, double rightStickX) {
        double y = -leftStickY * .3; // Remember, Y stick value is reversed
        double x = leftStickX * 1.1 * .3; // Counteract imperfect strafing
        double rx = rightStickX * .3;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;
        topLeftDriveMotor.setPower(frontLeftPower);
        bottomLeftDriveMotor.setPower(backLeftPower);
        topRightDriveMotor.setPower(frontRightPower);
        bottomRightDriveMotor.setPower(backRightPower);

    }

    public void armScoopMovement(Gamepad gamepad1, Gamepad gamepad2) throws InterruptedException {
        if (gamepad2.back) {
            droneLauncher.setPower(.5);
            sleep(500);
            sleep(500);
            droneLauncher.setPower(-.5);
            sleep(500);
            droneLauncher.setPower(0);
        } else {
            droneLauncher.setPower(0);
        }
        //KEEP LEFT SCOOP POSITIVE AND RIGHT NEGATIVE
        //**DO NOT CHANGE
        if (gamepad1.a) {
            //scoop down
            leftScoop.setPower(.8);
            rightScoop.setPower(-.8);
        }
        //KEEP RIGHT SCOOP POSITIVE AND LEFT NEGATIVE
        //**DO NOT CHANGE
        if (gamepad1.y) {
            //scoop up
            //ElapsedTime runtime = new ElapsedTime();
            leftScoop.setPower(-.8);
            rightScoop.setPower(.8);
            leftWheel.setPower(-1);
            rightWheel.setPower(1);
        }
        if (gamepad2.b) {
            //grab open
            claw.setPower(.6);
        }
        if (gamepad2.x) {
            claw.setPower(-.6);
            //close
        }
        if (gamepad1.b){
                //spins backwards
                 leftWheel.setPower(-1);
                rightWheel.setPower(1);

            }
        else if (gamepad1.x){
                //end
             leftWheel.setPower(0);
                rightWheel.setPower(0);

            }



        //move arm back
        if (gamepad2.left_trigger > 0) {
            armmotorTop.setPower(-.3 * gamepad2.left_trigger);
            armmotorBottom.setPower(-.6 * gamepad2.left_trigger);

        } else if (gamepad2.right_trigger > 0) {
            armmotorTop.setPower(.3 * gamepad2.right_trigger);
            //forward
            armmotorBottom.setPower(.6 * gamepad2.right_trigger);


        } else {
            armmotorTop.setPower(0);
            armmotorBottom.setPower(0);
        }

        if (gamepad2.left_bumper || gamepad2.dpad_up) {
            //back
            wrist.setPower(1);
        }
        if (gamepad2.right_bumper || gamepad2.dpad_down) {
            //front
            wrist.setPower(-1);
        }
        if (gamepad1.back) {
            double hang = 1.5;
            armmotorTop.setPower(hang);
            armmotorBottom.setPower(hang);
            sleep(10000000);



            }
        }



    }










