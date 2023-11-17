package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class test extends OpMode {

    private DcMotor testMotor;
    private CRServo testServo;

    @Override
    public void init() {
        testMotor = hardwareMap.get(DcMotor.class, "hang");
        testMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        testServo = hardwareMap.get(CRServo.class, "test");
    }

    @Override
    public void loop() {
        if (gamepad2.b) {
            testMotor.setPower(1);
        } else {
            testMotor.setPower(0);
        }

        if (gamepad2.a) {
            testServo.setPower(1);
        } else {
            testMotor.setPower(0);
        }
    }
}
