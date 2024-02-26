package org.firstinspires.ftc.teamcode.AutonomousOdometry;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OpenCV.LocationID;
import org.firstinspires.ftc.teamcode.OpenCV.OTAT2;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous
public class CloseRightAuto extends LinearOpMode {
    public String choice;
    public TrajectorySequence toPropMid, parkCornerOutside2,
            toPropRight, parkCornerOutside3,
            toPropLeft, parkCornerOutside1;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        LocationID id = new LocationID();
        OTAT2 detection = new OTAT2();
        VisionPortal portal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), id);
        Pose2d startPose =  new Pose2d(12, -61, Math.toRadians(90));
        drive.setPoseEstimate(startPose); // start for far left
        while (opModeInInit()){
            buildTrajs(startPose, drive);
            telemetry.addData("Zone", id.getSelection());
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

        // telemetry.addData("Zone", id.getSelection());
        //assume close left for rn

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()){
            telemetry.addData("choice: ", choice);
            telemetry.update();
            if (choice.equals("RIGHT")){
                drive.followTrajectorySequence(toPropRight);
                drive.armUp();
                drive.armDown();
                drive.followTrajectorySequence(parkCornerOutside3);
                //drive.followTrajectory(resetRight);
            }
            else if (choice.equals("LEFT")){
                drive.followTrajectorySequence(toPropLeft);
                drive.armUp();
                drive.armDown();
                drive.followTrajectorySequence(parkCornerOutside1);


            }
            else{
                drive.followTrajectorySequence(toPropMid);
                drive.armUp();
               drive.armDown();
                drive.followTrajectorySequence(parkCornerOutside2);


            }
            //drive.followTrajectorySequence(toPropLeftSeqRev);
            break;

        }

        //drive.followTrajectory(reverse);
        //drive.followTrajectory(reset);
    }
    public void buildTrajs(Pose2d startPose, SampleMecanumDrive drive){
        toPropMid = drive.trajectorySequenceBuilder(startPose) // working
                .forward(28)
                .back(5)
                .lineToLinearHeading(new Pose2d(49, -35, Math.toRadians(10)))
                //.splineToSplineHeading(new Pose2d(52, -36, Math.toRadians(0)), Math.toRadians(0))
                .build();
        parkCornerOutside2 = drive.trajectorySequenceBuilder(toPropMid.end()) // working 90 percent
                .strafeRight(23)
                .forward(9)
                .build();




        toPropRight = drive.trajectorySequenceBuilder(startPose) // working 90 percent
                .forward(14)
                .splineToLinearHeading(new Pose2d(15, -32, Math.toRadians(20)), Math.toRadians(0))
                //.splineToConstantHeading(new Vector2d(8, -38))
                .back(5)
                .strafeRight(13)
                .splineToSplineHeading(new Pose2d(48, -36.5, Math.toRadians(20)), Math.toRadians(0))
                .build();
        parkCornerOutside3 = drive.trajectorySequenceBuilder(toPropRight.end()) // working 90 percent
                //.back(5)
                .strafeRight(18)
                .forward(10)
                .build();


        toPropLeft = drive.trajectorySequenceBuilder(startPose)
                .forward(14)
                .splineToLinearHeading(new Pose2d(13, -30, Math.toRadians(180)), Math.toRadians(0))
                .back(7)
                .splineToSplineHeading(new Pose2d(51, -28.5, Math.toRadians(357)), Math.toRadians(0))
                .build();
        parkCornerOutside1 = drive.trajectorySequenceBuilder(toPropLeft.end()) // working 90 percent
                .strafeRight(29)
                .forward(9)
                .build();

    }

}


