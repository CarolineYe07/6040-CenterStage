package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous
public class BasicParkAuto extends LinearOpMode {

    private AprilTagProcessor tagProcessor;
    private VisionPortal visionPortal;


    @Override
    public void runOpMode() throws InterruptedException {
        Hardware robot = new Hardware(this);
        robot.init();
        telemetry.addLine("Robot Initialized");

        tagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), tagProcessor);
        telemetry.addLine("Vision Initialized");

        waitForStart();

        // turn to face boards to look at april tags
        // no odo (I cry)
        robot.setDrivePower(0.5, 0.5, 0.5, 0.5);
        sleep(500);
        robot.setDrivePower(0, 0, 0, 0);
        robot.setDrivePower(0.5, 0.5, 0, 0);
        sleep(500);
        robot.setDrivePower(0, 0, 0, 0);

        if (tagProcessor.getDetections().size() > 0) {
            AprilTagDetection tag = tagProcessor.getDetections().get(0);

            if (tag.ftcPose.y < 20) {
                robot.setDrivePower(1, 1, 1, 1);
                sleep(5000);
                robot.setDrivePower(0, 0, 0, 0);
            }
        }



    }
}
