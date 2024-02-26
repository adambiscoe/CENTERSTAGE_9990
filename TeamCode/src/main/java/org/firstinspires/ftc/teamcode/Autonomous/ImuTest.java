package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


/*Configuration file
Control Hub
I2C Port 08: imu

 */
@Disabled
@Autonomous
public class ImuTest extends LinearOpMode {
   BNO055IMU imu;


    //private double imuX = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle;
    //private double imuY = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle;
    ///private double imuZ = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;

    public void runOpMode() {
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);




        waitForStart();
        while (opModeIsActive()) {

         telemetry.addData("IMU test", botHeading);  // servo to usb ports, (vertical axes if control hub flat and facing you

            telemetry.update();

        }

    }

    /*public void initImu(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

    }
    public void imuTelemetry(){
        //if tilted left while parallel to table, roll of ch
        telemetry.addData("Pitch - x", "%.2f", imuX); // servo to usb ports, (vertical axes if control hub flat and facing you
        telemetry.addData("Roll - y", "%.2f", imuY); // motor to sensor ports, (horiz if control hub flat)
        telemetry.addData("Yaw - z", "%.2f", imuZ);
        telemetry.update();

    }
    */

}




