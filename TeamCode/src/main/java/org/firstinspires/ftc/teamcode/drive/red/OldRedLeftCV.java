package org.firstinspires.ftc.teamcode.drive.red;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(group="Red")
public class OldRedLeftCV extends LinearOpMode {


    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    private Servo claw, wrist;

    int step = 0;

    double clawPosition = 0;

    public static double armDownSpeed = 0.1;
    public static double armUpSpeed = -0.15;


    public static int targetPosition = 0;

    // Motors at the beginning of the arm
    private DcMotorEx belt = null; // Belt motor
    private DcMotorEx belt2 = null;

    static final double FEET_PER_METER = 3.28084;


    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;


    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        belt = hardwareMap.get(DcMotorEx.class, "belt");
        belt2 = hardwareMap.get(DcMotorEx.class, "belt2");

        belt2.setDirection(DcMotorEx.Direction.REVERSE);

        claw = hardwareMap.get(Servo.class, "claw");
        claw.setDirection(Servo.Direction.REVERSE);

        wrist = hardwareMap.get(Servo.class, "wrist");
        wrist.setDirection(Servo.Direction.REVERSE);

        belt.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        belt2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);

            }

            @Override
            public void onError(int errorCode) {

            }
        });


        telemetry.setMsTransmissionInterval(50);


        //need to put in hardware map

        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }


            telemetry.update();
            sleep(20);
        }


        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        //code
        claw.setPosition(0);
        wrist.setPosition(0.4);

        targetPosition = 0;

        Pose2d startPose = new Pose2d(-33.69, -68.17, Math.toRadians(90.00));
        drive.setPoseEstimate(startPose);

        Trajectory leftTraj1 = drive.trajectoryBuilder(startPose, false)
                .lineToConstantHeading(new Vector2d(-11, -63))
                .build();

        Trajectory leftTraj2 = drive.trajectoryBuilder(leftTraj1.end(), false)
                .lineToLinearHeading(new Pose2d(-10.5, -13, Math.toRadians(133.67)))
                .build();

        Trajectory leftTraj3 = drive.trajectoryBuilder(leftTraj2.end(), false)
                .forward(10)
                .build();

        Trajectory leftTraj4 = drive.trajectoryBuilder(leftTraj3.end(), false)
                .strafeLeft(1.125)
                .build();

        Trajectory leftTraj5 = drive.trajectoryBuilder(leftTraj4.end(), false)
                .lineToLinearHeading(new Pose2d(-13.5, -11, Math.toRadians(185)))
                .build();

        Trajectory leftTraj6 = drive.trajectoryBuilder(leftTraj5.end(), false)
                .lineToLinearHeading(new Pose2d(-50, -10.8, Math.toRadians(185)))
                .build();

        Trajectory leftTraj7 = drive.trajectoryBuilder(leftTraj6.end(), false)
                .lineToLinearHeading(new Pose2d(-11, -18, Math.toRadians(133.67)))
                .build();

        Trajectory leftTraj8 = drive.trajectoryBuilder(leftTraj7.end(), false)
                .lineToLinearHeading(new Pose2d(-19.182, -7.243, Math.toRadians(132.781)))
                .build();

    while (step == 0 && opModeIsActive()) {
        if (step == 0) {
                if (tagOfInterest.id == LEFT) {
                    grabCone();

                    sleep(500);

                    drive.followTrajectory(leftTraj1);

                    liftArm(570);

                    drive.followTrajectory(leftTraj2);
                    drive.followTrajectory(leftTraj3);

                    sleep(100);

                    drive.followTrajectory(leftTraj4);

                    sleep(1000);

                    releaseCone();

                    sleep(1000);

                    drive.followTrajectory(leftTraj5);

                    sleep(4000);

                    drive.followTrajectory(leftTraj6);

                    lowerArm(185);

                    requestOpModeStop();

                /*grabCone();

                sleep(1000);

                liftArm(570);

                sleep(750);

                drive.followTrajectory(leftTraj7);

                sleep(400);

                drive.followTrajectory(leftTraj8);*/
            }

                step++;
        } else {
            telemetry.addData("Waiting for: ", "STOP");
        }
    }

}

    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    public void liftArm(int target) {

        belt.setTargetPosition(target);
        belt2.setTargetPosition(target);

        belt.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        belt2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        belt.setPower(armUpSpeed);
        belt2.setPower(armUpSpeed);

        while (belt.isBusy() && belt2.isBusy()) {
            idle();
        }

    }

    public void lowerArm(int target) {
        targetPosition = 0;

        belt.setTargetPosition(target);
        belt2.setTargetPosition(target);

        belt.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        belt2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        belt.setPower(armDownSpeed);
        belt2.setPower(armDownSpeed);

        while (belt.isBusy() && belt2.isBusy()) {
            idle();
        }
    }

    public void grabCone() {
        wrist.setPosition(0.425);
        claw.setPosition(0.4);
    }

    public void releaseCone() {
        wrist.setPosition(0.425);
        claw.setPosition(0.15);
    }

}