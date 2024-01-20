package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Disabled
public class PowerPlayAutoByTime extends LinearOpMode {

    private AprilTagProcessor tagProcessor;
    private VisionPortal visionPortal;

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor fourBar;

    @Override
    public void runOpMode() throws InterruptedException {
        Hardware robot = new Hardware(this);
        fourBar = hardwareMap.get(DcMotor.class, "fourBar");

        robot.init();
        telemetry.addLine("Robot Initialized");
        telemetry.update();

        tagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), tagProcessor);

        double speed = 0.75;
        double forward_time = 0.5;
        double strafe_time = 0.75;
        double forward_speed = 0.5;

        waitForStart();

        fourBar.setPower(0.5);
        sleep(5000);

        if (opModeIsActive() && tagProcessor.getDetections().size() > 0) {
            telemetry.addLine("Tag seen");
            telemetry.update();
            AprilTagDetection tag = tagProcessor.getDetections().get(0);

            robot.setDrivePower(forward_speed, forward_speed, forward_speed, forward_speed);
            sleep(750);

            if (tag.id == 1) {
                telemetry.addLine("Tag 1 detected");
                telemetry.update();
                runtime.reset();

                while (runtime.seconds() < strafe_time) {
                    robot.setDrivePower(speed, -speed, speed, -speed);
                }
            } else if (tag.id == 2) {
                telemetry.addLine("Tag 3 detected");
                telemetry.update();
                runtime.reset();

                while (runtime.seconds() < strafe_time) {
                    robot.setDrivePower(-speed, speed, -speed, speed);
                }
            }
        }
        else {
            telemetry.addLine("No tag seen");
            telemetry.update();
        }
    }
}

