package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous

public class AutonomousFirst extends LinearOpMode {
    private Blinker control_Hub;
    private static DcMotorEx backleft;
    private static DcMotorEx backright;
    private static DcMotorEx frontleft;
    private static DcMotorEx frontright;
    private Gyroscope imu;
    public static CRServo claw;
    private CRServo wrist;
    public DcMotor armmotorTop;
    public DcMotor armmotorBottom;
    private CRServo leftScoop;
    private CRServo rightScoop;
    private CRServo rightWheel;
    private CRServo leftWheel;


    static final double COUNTS_PER_MOTOR_REV = 545;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 3.937;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;
    static final int RIGHT_ANGLE_POSITION = 884; // could change during more testing
    static final int HALF_ANGLE_POSITION = RIGHT_ANGLE_POSITION / 2;
    HardwareMap hwMap;

    static final double ticks = 530;
    // 530 ticks = 31.416 cm = 1.031 ft = 12.372 inch
    static final long rightAngleTime = 430;
    static final long halfTurnTime = 215;
    ElapsedTime runtime;

    //turn .5 power, 430 ms 90 degree, 215 ms 45 degree turn
    @Override
    // todo: write your code here
    public void runOpMode() {
        //initialize, reset encoder, run using encoder with enforced PID velocity
        initHardware(hwMap);
        runtime = new ElapsedTime();
        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            // this is for far left
            strafeLeftEncoder(1000, .5);



            /*

            telemetry.addData("updating front", dsensorFront.getDistance(DistanceUnit.INCH));
            telemetry.addData("updating left ", dsensorLeft.getDistance(DistanceUnit.INCH));
            telemetry.addData("updating right", dsensorRight.getDistance(DistanceUnit.INCH));
            telemetry.update();
            claw.setPower(.3);
            setPowers(.5);
            sleep(850);
            //move within 7 inches from tape, move around 25.5 inches
            // setPowers(.5);
            setPowers(0);
            propScan();
            setPowers(-.5, 850);
            strafeLeft(1500); // estimation
            setPowers(.5, 850);
            turnLeft(rightAngleTime);
            setPowers(.3, 500);
            armDown();
            armUp();
            break;
            */

        }
    }

    public void setPowers(double power) {
        backleft.setPower(power);
        backright.setPower(power);
        frontleft.setPower(power);
        frontright.setPower(power);
    }

    public void setPowers(double power, long time) {
        backleft.setPower(power);
        backright.setPower(power);
        frontleft.setPower(power);
        frontright.setPower(power);
        sleep(time);
        setPowers(0);
    }

    public void setVs(double velocity) {
        frontleft.setVelocity(velocity);
        frontright.setVelocity(velocity);
        backleft.setVelocity(velocity);
        backright.setVelocity(velocity);
    }

    public void setTPs(int TP) {
        frontleft.setTargetPosition(TP);
        frontright.setTargetPosition(TP);
        backleft.setTargetPosition(TP);
        backright.setTargetPosition(TP);
    }

    public void moveEncoder(double distance, double power) {
        //Boolean running = true;
        double target = distance * COUNTS_PER_INCH;
        setTPs((int) target);
        while (true) {
            setModes(DcMotor.RunMode.RUN_TO_POSITION);
            setPowers(power);
            if (isBusy()) {
                idle();
            } else {
                break;
            }

        }
        setPowers(0);
        setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }

    public void setModes(DcMotor.RunMode mode) {
        frontleft.setMode(mode);
        frontright.setMode(mode);
        backleft.setMode(mode);
        backright.setMode(mode);
    }


    public double getCurrentPosition(String motor) { // this isnt working rn
        if (motor.equals("fr")) {
            return frontright.getCurrentPosition();
        } else if (motor.equals("fl")) {
            return frontleft.getCurrentPosition();
        } else if (motor.equals("br")) {
            return backright.getCurrentPosition();
        } else if (motor.equals("br")) {
            return backleft.getCurrentPosition();
        } else {
            return 0.0;
        }

    }

    public void turnLeftEncoder(int target) {
        // just for right angles for now
        //dont know why theyre reversed, probably gearboxes

        frontright.setTargetPosition(-target);
        backright.setTargetPosition(-target);
        frontleft.setTargetPosition(target);
        backleft.setTargetPosition(target);
        while (true) {
            setModes(DcMotor.RunMode.RUN_TO_POSITION);
            setPowers(.5);
            if (isBusy()) {
                idle();
            } else {
                break;
            }
        }
        setPowers(0);
        setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void turnRightEncoder(int target) {
        // just for right angles for now
        //dont know why theyre reversed, probably gearboxes

        frontright.setTargetPosition(target);
        backright.setTargetPosition(target);
        frontleft.setTargetPosition(-target);
        backleft.setTargetPosition(-target);
        while (true) {
            setModes(DcMotor.RunMode.RUN_TO_POSITION);
            setPowers(.5);
            if (isBusy()) {
                idle();
            } else {
                break;
            }
        }
        setPowers(0);
        setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void strafeLeftEncoder(int distance, double power) {
        frontleft.setTargetPosition(-distance);
        frontright.setTargetPosition(distance);
        backleft.setTargetPosition(distance);
        backright.setTargetPosition(-distance);

        while (true) {

            setModes(DcMotor.RunMode.RUN_TO_POSITION);
            setPowers(power);
            if (isBusy()) {
                idle();
            } else {
                break;
            }

        }
        setPowers(0);
        setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //sleep(250);


    }

    public void strafeRightEncoder(int distance, double power) {
        frontleft.setTargetPosition(distance);
        frontright.setTargetPosition(-distance);
        backleft.setTargetPosition(-distance);
        backright.setTargetPosition(distance);

        while (true) {

            setModes(DcMotor.RunMode.RUN_TO_POSITION);
            setPowers(power);
            if (isBusy()) {
                idle();
            } else {
                break;
            }

        }
        setPowers(0);
        setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //sleep(250);


    }

    public void claw() {
        claw.setPower(1);
    }


    public void propDrop() {
        //may need to account for arm movment due to truss blocks
        /*
        claw.setPower(.7);// hold claw open
        sleep(1500);
        */
        scoopDown();
        sleep(1200);
        wrist.setPower(.6);// move wrist forwards in autonomus, CHANGE TO 1 MAYBe
        armmotorTop.setPower(-.3);
        armmotorBottom.setPower(-.6);
        sleep(1000); // potentially make it more time with new adjustments to claw
        clawOpen();
        armmotorTop.setPower(0);
        armmotorBottom.setPower(0);
        sleep(800);
         // close celaw to drop, MUST SET LIMITS SOON
        //claw.setPower(0); // stop claw
        armDown(); // includes arm reset
    }
    public void propDropYellow(){
        scoopDown();
        sleep(1200);
        wrist.setPower(.6);// move wrist forwards in autonomus, CHANGE TO 1 MAYBe
        armmotorTop.setPower(-.3);
        armmotorBottom.setPower(-.6);
        sleep(1000); // potentially make it more time with new adjustments to claw
        armmotorTop.setPower(0);
        armmotorBottom.setPower(0);
        sleep(500);
        // close celaw to drop, MUST SET LIMITS SOON
        //claw.setPower(0); // stop claw
        //armDown(); // includes arm reset

    }

    public void armDown() {
        //claw.setPower(1);
        //sleep(2000);
        clawOpen();
        sleep(1000);
        scoopDown();
        sleep(2000);
        wrist.setPower(-.75);
        armmotorBottom.setPower(.8);
        armmotorTop.setPower(.4);
        sleep(575);
        armmotorBottom.setPower(0);
        armmotorTop.setPower(0);

        claw.close();
        scoopUp();
        sleep(2000);
    }

    public void scoopUp() {
        //KEEP RIGHT SCOOP POSITIVE AND LEFT NEGATIVE
        //**DO NOT CHANGE
        //scoop up
        leftScoop.setPower(-.6);
        rightScoop.setPower(.6);
    }

    public void scoopDown() {
        //scoop down
        //KEEP LEFT SCOOP POSITIVE AND RIGHT NEGATIVE
        //**DO NOT CHANGE
        leftScoop.setPower(.8);
        rightScoop.setPower(-.8);
    }


    public void armUp() {
        clawPropClose();
        scoopDown();
        sleep(1500);
        armmotorBottom.setPower(-.6);
        armmotorTop.setPower(-.3);
        sleep(850);
        armmotorBottom.setPower(0);
        armmotorTop.setPower(0);
        wrist.setPower(.75);
        sleep(1000);
        //may need this next line
        moveEncoder(4, .5);
        clawOpen();
        sleep(1000);
    }
    public void clawClose(){
        claw.setPower(-1);
    }
    public void clawOpen(){
        claw.setPower(1);
    }
    public void clawPropOpen(){
        claw.setPower(.6);
    }
    public void clawPropClose(){
        claw.setPower(-.6);
    }




    public boolean isBusy() {
        if (frontleft.isBusy() || frontright.isBusy() || backright.isBusy() || backleft.isBusy()) {
            return true;

        } else {
            return false;
        }
    }


    public void initHardware(HardwareMap ahwMap) {
        hardwareMap = ahwMap;

        backleft = hardwareMap.get(DcMotorEx.class, "backleft");
        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backright = hardwareMap.get(DcMotorEx.class, "backright");
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontleft = hardwareMap.get(DcMotorEx.class, "frontleft");
        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright = hardwareMap.get(DcMotorEx.class, "frontright");
        rightWheel = hardwareMap.crservo.get("rightwheel");
        leftWheel = hardwareMap.crservo.get("leftwheel");
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftScoop = hardwareMap.crservo.get("leftScoop");
        rightScoop = hardwareMap.crservo.get("rightScoop");
        armmotorBottom = hardwareMap.dcMotor.get("ambottom");
        armmotorTop = hardwareMap.dcMotor.get("amtop");
        wrist = hardwareMap.crservo.get("wrist");
        claw = hardwareMap.crservo.get("claw");
        armmotorTop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armmotorBottom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armmotorTop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armmotorBottom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Zero Power Behavior Brakes
        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //leave only backleft reversed due to gearbox switchup
        backleft.setDirection(DcMotor.Direction.REVERSE);
        setPowers(0);
    }

    public void idleUntilDone(int power) {
        while (true) {

            setModes(DcMotor.RunMode.RUN_TO_POSITION);
            setPowers(power);
            if (isBusy()) {
                idle();
            } else {
                break;
            }

        }
        setPowers(0);
        setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


}
