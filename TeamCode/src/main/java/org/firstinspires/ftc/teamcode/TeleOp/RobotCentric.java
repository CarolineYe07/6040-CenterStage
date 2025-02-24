package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class RobotCentric extends LinearOpMode {

    private DcMotor lf;
    private DcMotor lb;
    private DcMotor rf;
    private DcMotor rb;

    private DcMotor arm;
    private Servo rotator;
    private Servo wrist;
    private CRServo clamp;
    private CRServo drone;
    private CRServo leftIntake;
    private CRServo rightIntake;
    private DcMotor leftHang;
    private DcMotor rightHang;

    private int armPos;

    private double speed_multiplier;

    @Override
    public void runOpMode() throws InterruptedException {

        lf = hardwareMap.get(DcMotor.class, "lf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        rf = hardwareMap.get(DcMotor.class, "rf");
        rb = hardwareMap.get(DcMotor.class, "rb");

        arm = hardwareMap.get(DcMotor.class, "arm");
        rotator = hardwareMap.get(Servo.class, "rotator");
        wrist = hardwareMap.get(Servo.class, "wrist");
        clamp = hardwareMap.get(CRServo.class, "clamp");
        drone = hardwareMap.get(CRServo.class, "drone");
        leftIntake = hardwareMap.get(CRServo.class, "leftIntake");
        rightIntake = hardwareMap.get(CRServo.class, "rightIntake");
        leftHang = hardwareMap.get(DcMotor.class, "leftHang");
        rightHang = hardwareMap.get(DcMotor.class, "rightHang");

        leftHang.setDirection(DcMotorSimple.Direction.REVERSE);


        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER );

        armPos = 0;
        wrist.setPosition(0.05);
        rotator.setPosition(0);
        boolean isOpen = false;


        lf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        waitForStart();


        while (!isStopRequested() && opModeIsActive()) {

            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // slow mode
            if (gamepad1.right_bumper) {
                speed_multiplier = 0.5;
            } else {
                speed_multiplier = 1;
            }

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator * speed_multiplier;
            double backLeftPower = (y - x + rx) / denominator * speed_multiplier;
            double frontRightPower = (y - x - rx) / denominator * speed_multiplier;
            double backRightPower = (y + x - rx) / denominator * speed_multiplier;


            lf.setPower(frontLeftPower);
            lb.setPower(backLeftPower);
            rf.setPower(frontRightPower);
            rb.setPower(backRightPower);


            // Orient pixels
            if(gamepad2.dpad_left){
                rotator.setPosition(.4);
            } else if(gamepad2.dpad_up){
                rotator.setPosition(0);
            } else if(gamepad2.dpad_right){
                rotator.setPosition(.75);
            }else if(gamepad2.dpad_down){
                rotator.setPosition(.5);
            }

            // Launch Drone
            if(gamepad2.x){
                drone.setPower(1);
            } else{
                drone.setPower(0);
            }

            // Run hang motors
            if (gamepad1.right_trigger > 0) {
                leftHang.setPower(.5);
                rightHang.setPower(.5);
            } else if (gamepad1.left_trigger > 0) {
                leftHang.setPower(-.5);
                rightHang.setPower(-.5);
            } else {
                leftHang.setPower(0);
                rightHang.setPower(0);
            }

            //Intake toggle
            if(gamepad2.y && isOpen){
                leftIntake.setPower(.5);
                rightIntake.setPower(-.5);
                isOpen = false;

            } else if(gamepad2.y && !isOpen) {
                leftIntake.setPower(-.5);
                rightIntake.setPower(.5);
                isOpen = true;
            }

            // Pixel release failsafe
            if(gamepad2.left_bumper){
                clamp.setPower(.5);
            }
            if(gamepad2.right_bumper){
                clamp.setPower(-.5);
            }

            // To adjust claw angle when placing on the Backdrop
            if(gamepad2.left_trigger > 0){

                wrist.setPosition(.4);
            }
            if(gamepad2.right_trigger > 0){

                wrist.setPosition(.6);
            }


            // Reset the arm and claw positions
            if(gamepad2.b){
                clamp.setPower(1);
                sleep(250);
                rotator.setPosition(0);
                sleep(250);
                wrist.setPosition(.05);

                //Pick up pixel and bring up.
            } else if(gamepad2.a){

                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER );
                clamp.setPower(-1);
                sleep(500);
                moveArm(-100, .35);
                sleep(500);
                wrist.setPosition(1);

                //When not in autonomous mode, the arm can be controlled manually for more specific heights.
            } else {

                telemetry.addLine("A and B are not pressed");
                telemetry.update();
                arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER );
                arm.setPower(gamepad2.left_stick_x * .5);
            }
        }

    }
    //Move arm to target position at a set speed. Once target position is reached, revert controls back to driver.
    private void moveArm(int armTarget, double speed) {

        armPos += armTarget;

        arm.setTargetPosition(armPos);

        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        arm.setPower(speed);


        while (opModeIsActive() && arm.isBusy()) {
            idle();
        }
    }
}