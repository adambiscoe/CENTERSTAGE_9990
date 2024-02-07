package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TestDrive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Drivetrain drivetrain = new Drivetrain();
        drivetrain.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()){
            //this is now working
            //drivetrain.claw.setPower(.5);
            if (gamepad1.b){
                //spins backwards
                drivetrain.rightWheel.setPower(1);
                drivetrain.leftWheel.setPower(-1);
            }
            if (gamepad1.x){
                //end
                drivetrain.rightWheel.setPower(0);
                drivetrain.leftWheel.setPower(0);
            }
            drivetrain.armScoopMovement(gamepad1, gamepad2);
            double lsy = gamepad1.left_stick_y;
            double lsx = gamepad1.left_stick_x;
            double rsx = gamepad1.right_stick_x;

            if (gamepad1.left_trigger > 0){
                drivetrain.moveRobot(.3* lsy, .3*  lsx, .3*  rsx);
                drivetrain.armScoopMovement(gamepad1, gamepad2);
            }
            else{
                drivetrain.moveRobot(lsy, lsx, rsx);
                drivetrain.armScoopMovement(gamepad1, gamepad2);
            }
        }
    }
}
