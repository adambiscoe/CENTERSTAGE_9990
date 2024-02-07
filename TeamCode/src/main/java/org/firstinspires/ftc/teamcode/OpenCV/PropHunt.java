package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Autonomous.AutonomousFirst;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
public class PropHunt extends LinearOpMode {
    private String choice;
    static final int RIGHT_ANGLE_POSITION = 896; // could change during more testing
    static final int HALF_ANGLE_POSITION = RIGHT_ANGLE_POSITION / 2;
    static final int PROP_ANGLE_POSITION = 500;
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
            /*
            telemetry.addData("choice: ", choice);
            telemetry.update();
            robot.moveEncoder(23, .5);
            if (choice.equals("RIGHT")){
                robot.armDown(); // this doesnt work, make something to make the arm move up before placing pixel
                sleep(1000);
                robot.turnRightEncoder(PROP_ANGLE_POSITION);
                robot.moveEncoder(-2, .5);
                robot.propDrop();
                robot.turnLeftEncoder(PROP_ANGLE_POSITION);
            }
            else if (choice.equals("LEFT")){
                robot.armDown();
                sleep(1000);
                robot.turnLeftEncoder(PROP_ANGLE_POSITION);
                robot.moveEncoder(-2, .5);
                robot.propDrop();
                robot.turnRightEncoder(PROP_ANGLE_POSITION);
            }
            else{
                robot.moveEncoder(-2, .5);
                robot.propDrop();
            }
            // from this point on, becomes dependent on which quadrant you are starting in
            //assume close left for testing
            robot.moveEncoder(-17, .5);
            robot.turnLeftEncoder(RIGHT_ANGLE_POSITION);
            robot.moveEncoder(33, .5);
            robot.strafeLeftEncoder(1000/5, .4);
            robot.moveEncoder(4, .5);
            /*
            robot.strafeRightEncoder(1000,.5);
            robot.armDown();
            robot.armUp();
            sleep(2000);
            robot.strafeLeftEncoder(1000, .5);
            robot.moveEncoder(8, .5);

             */

           /* if (detection.getDesiredTagId() == 1){

            }

            */

           // robot.turnLeftEncoder(RIGHT_ANGLE_POSITION);
            //robot.moveEncoder(15, .5);

            // move infront of gameboard, move far enough to see every tag
            //detection.setDesiredTag(2);
            //detection.runOpMode();
            /*
            detection.initAprilTag();
            detection.setManualExposure(6, 250);


            Boolean tagFound = false;
            while (true){
                tagFound = detection.lookForTag();
                if(tagFound){
                    detection.driveToTag();
                    while (robot.isBusy()){
                        idle();
                    }
                    break;
                }
                */
            }
        robot.claw.setPower(1);















                    //configured properly

        }

    }

