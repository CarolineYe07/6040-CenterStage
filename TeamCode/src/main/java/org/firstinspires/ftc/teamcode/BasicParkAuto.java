package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous
public class BasicParkAuto extends LinearOpMode {

    private AprilTagProcessor tagProcessor;
    private VisionPortal visionPortal;

    private ElapsedTime runtime;

    @Override
    public void runOpMode() throws InterruptedException {
        Hardware robot = new Hardware(this);
        robot.init();

        runtime = new ElapsedTime();

        waitForStart();

        runtime.reset();

        while (opModeIsActive() && runtime.milliseconds() < 500) {
            robot.setDrivePower(0.5, -0.5, -0.5, 0.5);
        }
        robot.setDrivePower(0, 0, 0,0);


    }
}
