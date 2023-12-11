package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class test extends OpMode {

    private DcMotor myBeloved;

    @Override
    public void init() {
        myBeloved = hardwareMap.get(DcMotor.class, "myBeloved");
        myBeloved.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    @Override
    public void loop() {
        if (gamepad2.left_bumper) {
            myBeloved.setPower(0.5);
        } else if (gamepad2.right_bumper) {
            myBeloved.setPower(-0.5);
        } else {
            myBeloved.setPower(0);
        }
    }
}
