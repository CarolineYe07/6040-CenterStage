package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
public class ControlsTest extends OpMode {

    private DcMotor armMotor;
    private Servo rotator;

    @Override
    public void init() {
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rotator = hardwareMap.get(Servo.class, "clawRotator");
    }

    @Override
    public void loop() {
        if (gamepad2.left_bumper) {
            armMotor.setPower(0.5);
        } else if (gamepad2.right_bumper) {
            armMotor.setPower(-0.5);
        } else {
            armMotor.setPower(0);
        }

        if(gamepad2.dpad_up){
            rotator.setPosition(.5);
            telemetry.addData("dpad_up", 0.5);

        } else if(gamepad2.dpad_left){
            rotator.setPosition(0);
            telemetry.addData("dpad_left", 0);

        } else if(gamepad2.dpad_right){
            rotator.setPosition(1);
            telemetry.addData("dpad_right", 1);

        } else if (gamepad2.dpad_down) {
            rotator.setPosition(.75);
            telemetry.addData("dpad_down", 0.75);
        }

        telemetry.update();
    }
}
