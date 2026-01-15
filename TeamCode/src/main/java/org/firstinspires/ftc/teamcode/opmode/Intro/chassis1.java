package org.firstinspires.ftc.teamcode.opmode.Intro;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="chassis", group="intro")

public class chassis1 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor leftMotor = hardwareMap.dcMotor.get("left");
        DcMotor rightMotor = hardwareMap.dcMotor.get("right");
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);



        waitForStart();



        while (opModeIsActive()) {



            if (gamepad1.left_stick_y > 0.1 ) {
                leftMotor.setPower(gamepad1.left_stick_y);
                rightMotor.setPower(gamepad1.left_stick_y);

            }
            else{
                leftMotor.setPower(0);
                rightMotor.setPower(0);
            }

            if (gamepad1.left_stick_y < 0.1) {
                leftMotor.setPower(gamepad1.left_stick_y);
                rightMotor.setPower(gamepad1.left_stick_y);
            }
            else{
                leftMotor.setPower(0);
                rightMotor.setPower(0);
            }

        }
    }
}
