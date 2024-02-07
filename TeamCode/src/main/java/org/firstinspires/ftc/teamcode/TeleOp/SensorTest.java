package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class SensorTest extends LinearOpMode {
    private DistanceSensor dsensorFront;
    private DistanceSensor dsensorRight;
    private DistanceSensor dsensorLeft;
    public void runOpMode(){
        dsensorFront = hardwareMap.get(DistanceSensor.class, "ds");
        dsensorLeft = hardwareMap.get(DistanceSensor.class, "dsl");
        dsensorRight = hardwareMap.get(DistanceSensor.class, "dsr");
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("updating front", dsensorFront.getDistance(DistanceUnit.INCH));
            telemetry.addData("updating left ", dsensorLeft.getDistance(DistanceUnit.INCH));
            telemetry.addData("updating right", dsensorRight.getDistance(DistanceUnit.INCH));
            telemetry.update();

        }

    }


}
