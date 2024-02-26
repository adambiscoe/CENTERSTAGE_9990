package org.firstinspires.ftc.teamcode.AutonomousNew;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Autonomous.AutonomousFirst;
import org.firstinspires.ftc.teamcode.OpenCV.LocationID;
import org.firstinspires.ftc.teamcode.OpenCV.OTAT2;
import org.firstinspires.ftc.vision.VisionPortal;
@Disabled
@Autonomous
public class CloseRight extends LinearOpMode {
    private String choice;
    static final int RIGHT_ANGLE_POSITION = 880; // could change during more testing
    static final int HALF_ANGLE_POSITION = RIGHT_ANGLE_POSITION / 2;
    static final int PROP_ANGLE_POSITION = 600;
    static final int STRAFE_FULL_SQUARE = 1000;
    @Override
    public void runOpMode() throws InterruptedException {
        LocationID id = new LocationID();
        AutonomousFirst robot = new AutonomousFirst();
        OTAT2 detection = new OTAT2();
        VisionPortal portal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), id);
        telemetry.clear();
        while(opModeInInit()) {
            telemetry.addData("Zone", id.getSelection());
            robot.initHardware(hardwareMap);
            //assume close left for rn
            if (id.getSelection() == LocationID.Selected.LEFT){
                choice = "LEFT";
                detection.setDesiredTagId(1);
            }
            else if (id.getSelection() == LocationID.Selected.RIGHT){
                choice = "RIGHT";
                detection.setDesiredTagId(3);
            }
            else{
                choice = "MIDDLE";
                detection.setDesiredTagId(2);
            }
            telemetry.update();
        }
        while (opModeIsActive()){
            telemetry.addData("choice: ", choice);
            telemetry.update();
            robot.clawPropClose();
            robot.moveEncoder(23, .5);
            if (choice.equals("RIGHT")){


                robot.turnRightEncoder(PROP_ANGLE_POSITION + 100);
                //robot.moveEncoder(-2, .5);
                robot.propDrop();
                robot.moveEncoder(-1, .5);
                robot.turnLeftEncoder(PROP_ANGLE_POSITION + 100);
                robot.moveEncoder(-17, .7);
                robot.turnRightEncoder(RIGHT_ANGLE_POSITION);
                robot.moveEncoder(34, .7);
                robot.strafeLeftEncoder(STRAFE_FULL_SQUARE - 200, .5);
                //robot.moveEncoder(4, .5);
                robot.armUp();
                robot.strafeRightEncoder(STRAFE_FULL_SQUARE - 200,.5);
                robot.moveEncoder(8, .5);


            }
            else if (choice.equals("LEFT")){

                robot.turnLeftEncoder(PROP_ANGLE_POSITION + 50);
                // robot.moveEncoder(-2, .5);
                robot.propDrop();
                robot.moveEncoder(-1, .5);
                robot.turnRightEncoder(PROP_ANGLE_POSITION + 50 + RIGHT_ANGLE_POSITION);
                robot.moveEncoder(29, .7);
                robot.strafeLeftEncoder(400, .5);
                //robot.claw.();
                robot.armUp();
                sleep(500);
                robot.strafeRightEncoder(STRAFE_FULL_SQUARE + 400, .5);
                robot.moveEncoder(8, .5);


            }
            else{
                //robot.moveEncoder(-2, .5);
                // robot.claw.setPower(1);

                robot.strafeLeftEncoder(100, .5);
                robot.propDrop();
                robot.strafeRightEncoder(100, .5);
                robot.moveEncoder(-1, .5);
                robot.turnRightEncoder(RIGHT_ANGLE_POSITION);
                //robot.armDown(); called inside propdrop
                robot.moveEncoder(32, .7);
                robot.strafeLeftEncoder(200, .5);
                // robot.strafeRightEncoder(STRAFE_FULL_SQUARE, .7);
                //robot.moveEncoder(4, .5);
                robot.armUp();
                //robot.moveEncoder(-2, .5);
                robot.strafeRightEncoder(STRAFE_FULL_SQUARE + 200, .5);
                robot.moveEncoder(8, .5);
            }
            // from this point on, becomes dependent on which quadrant you are starting in
            //assume close left for testing
            /* THE FOLLOWING IS ONLY IF YOU JUST WANT TO PLACE PROP THEN PARK
            robot.moveEncoder(-17, .5);
            robot.turnLeftEncoder(RIGHT_ANGLE_POSITION);
            robot.moveEncoder(33, .5);
            robot.strafeLeftEncoder(1000/5, .4);
            robot.moveEncoder(4, .5);
             */
            robot.scoopDown();
            robot.armDown();
            break;
        }
    }}

