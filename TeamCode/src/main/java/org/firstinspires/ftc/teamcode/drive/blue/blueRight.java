package org.firstinspires.ftc.teamcode.drive.blue;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class blueRight extends LinearOpMode {
        @Override
        public void runOpMode() {
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

            Pose2d startPose = new Pose2d(-33.6, 60, 180);
            drive.setPoseEstimate(startPose);

            Trajectory traj1 = drive.trajectoryBuilder(startPose, false)
                    .lineToLinearHeading(new Pose2d(-13.2,57.8, Math.toRadians(0)))
                    .lineToLinearHeading(new Pose2d(-13.2,13.6, Math.toRadians(-135)))
                    .build();

            // put cone down code goes here

            // now go and pick up another code
            Trajectory traj2 = drive.trajectoryBuilder(traj1.end(), false)
                    .lineToLinearHeading(new Pose2d(-56.8,14, Math.toRadians(-180)))
                    .addDisplacementMarker(() -> {
                        // code to pick up new cone
                    })
                    .lineToLinearHeading(new Pose2d(-31.2,11.6, Math.toRadians(-63)))
                    .build();

            // put code here that puts new cone down

            Trajectory traj3 = drive.trajectoryBuilder(traj2.end(), false)
                    .splineToSplineHeading(new Pose2d(-36.4,38, Math.toRadians(84)), Math.toRadians(84))
                    .build();

            waitForStart();

            if(isStopRequested()) return;

            drive.followTrajectory(traj1);
            // put old cone down
            drive.followTrajectory(traj2);
            // put new cone down
            drive.followTrajectory(traj3);
        }

}
