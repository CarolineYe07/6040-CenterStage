package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware.Hardware;

@Disabled
public class BasicParkAutoRed extends LinearOpMode {


    // private ElapsedTime runtime;

    @Override
    public void runOpMode() throws InterruptedException {
        Hardware robot = new Hardware(this);
        robot.init();

        DcMotor intake = hardwareMap.dcMotor.get("intake");

        // runtime = new ElapsedTime();

        waitForStart();

        // runtime.reset();

        while (opModeIsActive()) {
            robot.setDrivePower(0.5, 0.5, 0.5, 0.5);
            sleep(100);
            robot.setDrivePower(0, 0, 0,0);

            robot.setDrivePower(0.5, -0.5, -0.5, 0.5);
            sleep(3000);
            robot.setDrivePower(0,0,0,0);

            intake.setPower(0.5);
            sleep(500);
            intake.setPower(0);
            break;
        }

        /**
        while (opModeIsActive() && runtime.seconds() < 1) {
            robot.setDrivePower(0.5, -0.5, -0.5, 0.5);
        }
        robot.setDrivePower(0, 0, 0,0);


        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 5) {
            robot.setDrivePower(0.5, -0.5, -0.5, 0.5);
        }
        robot.setDrivePower(0, 0, 0,0);
        **/

    }
}
