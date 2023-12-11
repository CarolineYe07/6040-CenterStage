package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp
public class FieldCentricExternalHardware extends LinearOpMode {

    private AprilTagProcessor tagProcessor;
    private VisionPortal visionPortal;

    private double SPEED_MULTIPLIER;
    private boolean SLOW_MODE;

    @Override
    public void runOpMode() throws InterruptedException {

        Hardware robot = new Hardware(this);

        robot.init();

        // attempting to write some code that sets the max speed lower when near april tags on the board
        tagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), tagProcessor);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        SPEED_MULTIPLIER = 0.9;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator * SPEED_MULTIPLIER;
            double backLeftPower = (rotY - rotX + rx) / denominator * SPEED_MULTIPLIER;
            double frontRightPower = (rotY - rotX - rx) / denominator * SPEED_MULTIPLIER;
            double backRightPower = (rotY + rotX - rx) / denominator * SPEED_MULTIPLIER;


            robot.setDrivePower(frontLeftPower, backLeftPower, frontRightPower, backRightPower);


            if (gamepad1.left_bumper) {
                SPEED_MULTIPLIER = 0.5;
            } else {
                SPEED_MULTIPLIER = 0.9;
            }

            // hang motor
            if (gamepad2.right_trigger > 0) {
                robot.hangMotorPower(1);
            } else if (gamepad2.left_trigger > 0) {
                robot.hangMotorPower(-1);
            } else {
                robot.hangMotorPower(0);
            }


            // hang servo
            if (gamepad2.a) {
                robot.hook.setPower(1);
            } else {
                robot.hook.setPower(0);
            }


            // scuff intake


            // drone launch
            if (gamepad2.x) {
                robot.setDroneShooter(1);
            }

        }
    }
}
