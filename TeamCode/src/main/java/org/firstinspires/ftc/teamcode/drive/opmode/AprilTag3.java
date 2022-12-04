/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.drive.opmode;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;


import java.util.ArrayList;

@Autonomous(name = "Autonomous3", group = "Concept")
public class AprilTag3 extends LinearOpMode
{
    //movement
    private static final double COUNTS_PER_MOTOR_REV = 537.7; //Ticks per rotation for the GoBilda 5202 PLanetary Motor
    private static final double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    private static final double WHEEL_DIAMETER_INCHES = 3.54331;     // For figuring circumference
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    static final double FEET_PER_METER = 3.28084;
    private double lastError = 0;
    ElapsedTime timer = new ElapsedTime();

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
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
    public void runOpMode()
    {
        PIDForMotor pid = new PIDForMotor();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startingPose = new Pose2d();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if(tagOfInterest.id == LEFT){
            TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startingPose)
                    .lineToLinearHeading(new Pose2d(10, -50, Math.toRadians(-90)))
                    .build();
            drive.followTrajectorySequence(trajSeq);
        }else if(tagOfInterest.id == MIDDLE){
            TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startingPose)
                    .strafeLeft(26)
                    .forward(50)
                    .strafeRight(14.5)
                    .forward(3)
                    .addDisplacementMarker(2, () -> {
                        drive.clawServo.setPosition(0.5);
                        // This marker runs 20 inches into the trajectory
                        // Run your action in here!
                    })
                    .addTemporalMarker(() -> drive.liftMotor.setPower(1))
                    .waitSeconds(3)
                    .forward(3)
                    .addTemporalMarker(() -> drive.liftMotor.setPower(-0.2))
                    .waitSeconds(.75)
                    .addTemporalMarker(() -> {
                        drive.clawServo.setPosition(0.3);
                    })
                    .back(4)
                    .addTemporalMarker(() -> {
                        drive.liftMotor.setPower(0);
                    })
                    .strafeRight(15)
                    .turn(Math.toRadians(-90))
                    .forward(10)
                    .build();
            drive.followTrajectorySequence(trajSeq);
        }else if(tagOfInterest.id == RIGHT){
            TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startingPose)
                    .strafeRight(25)
                    .forward(50)
                    .strafeLeft(25)
                    .turn(Math.toRadians(48))
                    .forward(5)
                    .addDisplacementMarker(2, () -> {
                        drive.clawServo.setPosition(0.5);
                        // This marker runs 20 inches into the trajectory

                        // Run your action in here!
                    })
                    .waitSeconds(3)
                    .forward(3)
                    .addTemporalMarker(() -> {
                        drive.clawServo.setPosition(0.3);
                    })
                    .back(10)
                    .addTemporalMarker(() -> {
                        drive.liftMotor.setPower(0);
                    })
                    .turn(Math.toRadians(-48))
                    .strafeRight(25)
                    .build();
            drive.followTrajectorySequence(trajSeq);
        }


        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {sleep(20);}
    }

    public void lift() {
        ElapsedTime timer = new ElapsedTime();

        double lastError = 0;
        double integralSum = 0;

        double Kp = 0.03;
        double Ki = 0.0;
        double Kd = 0.0002;
        double Kg = 0.05;
    }

    public void liftToPosition(SampleMecanumDrive drive, int target) {
        int currentPosition = drive.liftMotor.getCurrentPosition();
        while(Math.abs(currentPosition - target) > 6) {
            currentPosition = drive.liftMotor.getCurrentPosition();
            int targetPosition = target;
            double power = returnPower(targetPosition, drive.liftMotor.getCurrentPosition());
            drive.liftMotor.setPower(power);
            telemetry.addData("current position", currentPosition);
            telemetry.addData("targetPosition", targetPosition);
            telemetry.update();
        }
    }

    public double returnPower(double reference, double state) {
        double error = reference - state;
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        double output = (error * 0.03) + (derivative * 0.0002) + 0.05;
        return output;
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }


}