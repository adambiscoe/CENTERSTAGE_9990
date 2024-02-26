package org.firstinspires.ftc.teamcode.AutonomousOdometry;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
public class SplitSecond extends LinearOpMode {
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
        Pose2d startPose =  new Pose2d(-31, -61, Math.toRadians(90));
        drive.setPoseEstimate(startPose); // start for far left
        //Example trajectory from spline test
        /*
        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(30, 30), 0)
                .build();

        TrajectorySequence toPropMid = drive.trajectorySequenceBuilder(startPose) // working
                .forward(28)
                .back(21)
                .lineToLinearHeading(new Pose2d(-31, 58.5, Math.toRadians(358)))
                .forward(52)
                .splineToSplineHeading(new Pose2d(48.5, 35, Math.toRadians(0)), Math.toRadians(0))
                .build();
        TrajectorySequence parkCornerLeft2 = drive.trajectorySequenceBuilder(toPropMid.end()) // working 90 percent
                .strafeLeft(23)
                .forward(9)
                .build();




        TrajectorySequence toPropRight = drive.trajectorySequenceBuilder(startPose) // working 90 percent
                .splineToLinearHeading(new Pose2d(-32, 38, Math.toRadians(250)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-31, 58.5, Math.toRadians(355)), Math.toRadians(0))
                .forward(52)
                .splineToSplineHeading(new Pose2d(47, 25, Math.toRadians(355)), Math.toRadians(0))
                .build();
        TrajectorySequence parkCornerLeft3 = drive.trajectorySequenceBuilder(toPropRight.end()) // working 90 percent
                //.back(5)
                .lineTo(new Vector2d(35, 36))
                .splineToConstantHeading(new Vector2d(62, 51), Math.toRadians(350))
                .build();


        TrajectorySequence toPropLeft = drive.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(new Pose2d(-30, 38, Math.toRadians(0)), Math.toRadians(0))
                .back(14)
                .splineToLinearHeading(new Pose2d(-31, 58.5, Math.toRadians(357)), Math.toRadians(0))
                .forward(52)
                .splineToSplineHeading(new Pose2d(47, 39, Math.toRadians(0)), Math.toRadians(0))
                .build();
        TrajectorySequence parkCornerLeft1 = drive.trajectorySequenceBuilder(toPropLeft.end()) // working 90 percent
                .strafeLeft(13)
                .forward(8)
                .build();

         */

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
                //sleep(4000);
                drive.followTrajectorySequence(toPropRight);
                //drive.armUp();
                //drive.armDown();
                //drive.followTrajectorySequence(parkCornerOutside3);
                //drive.followTrajectory(resetRight);
            }
            else if (choice.equals("LEFT")){
                //sleep(4000);

                drive.followTrajectorySequence(toPropLeft);
                //drive.armUp();
                //drive.armDown();
                //drive.followTrajectorySequence(parkCornerOutside1);


            }
            else{
                //sleep(4000);

                drive.followTrajectorySequence(toPropMid);
                //drive.followTrajectory(reverse);
                //drive.armUp();
                //drive.armDown();
                //drive.followTrajectorySequence(parkCornerOutside2);


            }
            //drive.followTrajectorySequence(toPropLeftSeqRev);
            break;

        }

        //drive.followTrajectory(reverse);
        //drive.followTrajectory(reset);
    }
    public void buildTrajs(Pose2d startPose, SampleMecanumDrive drive){
        toPropMid = drive.trajectorySequenceBuilder(startPose) // working
                .forward(27)
                .back(24)
                //.lineToLinearHeading(new Pose2d(-31, -58.5, Math.toRadians(0)))
                //.forward(52)
                //.splineToSplineHeading(new Pose2d(52, -36, Math.toRadians(0)), Math.toRadians(0))
                .build();
        parkCornerOutside2 = drive.trajectorySequenceBuilder(toPropMid.end()) // working 90 percent
                .strafeRight(24)
                .forward(9)
                .build();




        toPropRight = drive.trajectorySequenceBuilder(startPose)
                //TODO: NOT WORKING
                .forward(14)
                .splineToLinearHeading(new Pose2d(-29, -26, Math.toRadians(0)), Math.toRadians(0))
                .back(14)
                .lineToLinearHeading(new Pose2d(-31, -57.5, Math.toRadians(9)))
                //.lineTo(new Vector2d(20, -57.5))
                //.forward(52)
                //.splineToSplineHeading(new Pose2d(52, -24, Math.toRadians(20)), Math.toRadians(0))

                .build();
        parkCornerOutside3 = drive.trajectorySequenceBuilder(toPropRight.end()) // working 90 percent
                //.back(5)
                //.lineTo(new Vector2d(35, 36))
                //.splineToConstantHeading(new Vector2d(62, 51), Math.toRadians(350)) not right direction
                .strafeRight(10)
                .forward(8)
                .build();


        toPropLeft = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-32, -38, Math.toRadians(160)), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(-40, -48))
                .splineToLinearHeading(new Pose2d(-31, -57.5, Math.toRadians(0)), Math.toRadians(0))
                //.forward(52)
                //.splineToSplineHeading(new Pose2d(52, -25, Math.toRadians(0)), Math.toRadians(0))
                .build();
        parkCornerOutside1 = drive.trajectorySequenceBuilder(toPropLeft.end()) // working 90 percent
                .strafeRight(30)
                .forward(12)
                // .turn(20)
                //.forward(3)
                .build();

    }

}


