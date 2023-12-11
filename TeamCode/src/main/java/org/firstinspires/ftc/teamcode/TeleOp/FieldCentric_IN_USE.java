package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class FieldCentric_IN_USE extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        double SPEED_COEFF = 1.0;

        // Declare our motors
        // Make sure your ID's match your configuration
        //TouchSensor touch = hardwareMap.get(TouchSensor.class, "Touch");
        DcMotor lf = hardwareMap.dcMotor.get("lf");
        DcMotor lb = hardwareMap.dcMotor.get("lb");
        DcMotor rf = hardwareMap.dcMotor.get("rf");
        DcMotor rb = hardwareMap.dcMotor.get("rb");

        DcMotor intake = hardwareMap.dcMotor.get("intake");
        Servo intakeLeftServo = hardwareMap.servo.get("intakeLeft");
        Servo intakeRightServo = hardwareMap.servo.get("intakeRight");

        DcMotor lift = hardwareMap.dcMotor.get("lift");
        //DcMotor rightLift = hardwareMap.dcMotor.get("rightLift");

        DcMotor hang = hardwareMap.dcMotor.get("hang");
        Servo drone = hardwareMap.get(Servo.class, "drone");
        drone.setDirection(Servo.Direction.REVERSE);
        CRServo hook = hardwareMap.get(CRServo.class, "hook");

        // The holding and release of pixels
        CRServo clawArm = hardwareMap.get(CRServo.class, "clawArm");
        Servo lClamp = hardwareMap.get(Servo.class, "lClamp");
        Servo rClamp = hardwareMap.get(Servo.class, "rClamp");
        Servo rotator = hardwareMap.get(Servo.class, "clawRotator");
        Servo ClawAdjuster = hardwareMap.get(Servo.class, "rightClawAdjuster");


        //Servo middleGate = hardwareMap.get(Servo.class, "middleGate");


        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        lf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rClamp.setDirection(Servo.Direction.REVERSE);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Retrieve the IMU from the hardware map
        BHI260IMU imu = hardwareMap.get(BHI260IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        BHI260IMU.Parameters parameters = new BHI260IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        rotator.setPosition(1);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            telemetry.update();

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }

            // slow mode
            if (gamepad1.left_bumper) {
                SPEED_COEFF = 0.5;
            } else {
                SPEED_COEFF = 1;
            }

            // Driver 2 Controls

            //Rotator to position on backdrop
            /**
            if(gamepad2.dpad_up){
                rotator.setPosition(.5);
            } else if(gamepad2.dpad_left){
                rotator.setPosition(0);
            } else if(gamepad2.dpad_right){
                rotator.setPosition(1);
            }
             **/

            //Clamp for pixels
            if(gamepad2.left_bumper){
                lClamp.setPosition(0);
                rClamp.setPosition(0);
            } else if(gamepad2.right_bumper){
                lClamp.setPosition(1);
                rClamp.setPosition(1);
            }

            //Intake
            if (gamepad1.left_trigger > 0) {
                intake.setPower(0.5);
            } else if (gamepad1.right_trigger > 0) {
                intake.setPower(-1);
            }

            if (gamepad2.left_trigger > 0) {
                intake.setPower(0.5);
            } else if (gamepad2.right_trigger > 0) {
                intake.setPower(-1);
            }

            if (gamepad2.left_trigger <= 0 && gamepad2.right_trigger <= 0 && gamepad1.left_trigger <= 0 && gamepad1.right_trigger <= 0) {
                intake.setPower(0);
            }


            if (gamepad2.dpad_up) {
                intakeLeftServo.setPosition(1);
                intakeRightServo.setPosition(1);
            } else if (gamepad2.dpad_down) {
                intakeLeftServo.setPosition(-0.4);
                intakeRightServo.setPosition(-0.4);
            } else {
                intakeLeftServo.setPosition(0.5);
                intakeRightServo.setPosition(0.5);
            }

            //Lift
            lift.setPower(Range.clip(gamepad2.right_stick_y, -0.5, 0.5));

            // Claw arm
            clawArm.setPower(gamepad2.left_stick_y);

            //Hang
            if(gamepad2.a){
                hang.setPower(1);
            } else if(gamepad2.b){
                hang.setPower(-1);
            } else {
                hang.setPower(0);
            }

            //Hang extension
            if(gamepad2.x){
                hook.setPower(.5);
            } else if(gamepad2.y){
                hook.setPower(-.5);
            }

            //Drone
            if(gamepad1.x){
                drone.setPosition(1);
            }

            // spin-take up and down



            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator * SPEED_COEFF;
            double backLeftPower = (rotY - rotX + rx) / denominator * SPEED_COEFF;
            double frontRightPower = (rotY - rotX - rx) / denominator * SPEED_COEFF;
            double backRightPower = (rotY + rotX - rx) / denominator * SPEED_COEFF;

            lf.setPower(frontLeftPower);
            lb.setPower(backLeftPower);
            rf.setPower(frontRightPower);
            rb.setPower(backRightPower);
        }
    }
}
