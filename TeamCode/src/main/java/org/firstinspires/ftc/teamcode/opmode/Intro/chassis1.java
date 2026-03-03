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

        double leftStickY = 0;


        waitForStart();


        while (opModeIsActive()) {

            leftStickY = gamepad1.left_stick_y;


            if (leftStickY>0.1 || leftStickY<0.1){
                leftMotor.setPower(leftStickY);
                rightMotor.setPower(leftStickY);
            }

            if (gamepad1.right_stick_x > 0.1){
                leftMotor.setPower(gamepad1.right_stick_x);
                rightMotor.setPower(gamepad1.right_stick_x * -1);

            }

            if (gamepad1.right_stick_x < -0.1){
                leftMotor.setPower(gamepad1.right_stick_x);
                rightMotor.setPower(gamepad1.right_stick_x * -1);
            }else {
                leftMotor.setPower(0);
                rightMotor.setPower(0);
            }


        }
    }
}
