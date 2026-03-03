package org.firstinspires.ftc.teamcode.opmode.Intro;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="chassis2", group="intro")

public class chassis2 extends LinearOpMode {

    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor backRight;
    DcMotor backLeft;


    @Override
    public void runOpMode() throws InterruptedException {

        frontRight = hardwareMap.get(DcMotor.class, "fr");
        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        backLeft = hardwareMap.get(DcMotor.class, "bl");
        backRight = hardwareMap.get(DcMotor.class, "br");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);


        waitForStart();


        while (opModeIsActive()) {

            double rightStickInput = gamepad1.right_stick_x;
            double leftStickInput = gamepad1.left_stick_y;

            if (leftStickInput > 0.1) {
                frontRight.setPower(leftStickInput);
                frontLeft.setPower(leftStickInput);
                backRight.setPower(leftStickInput);
                backLeft.setPower(leftStickInput);
            }
            if (leftStickInput < 0.1) {
                frontRight.setPower(leftStickInput);
                frontLeft.setPower(leftStickInput);
                backRight.setPower(leftStickInput);
                backLeft.setPower(leftStickInput);

            }
            if (rightStickInput > 0.1) {
                frontRight.setPower(rightStickInput);
                frontLeft.setPower(rightStickInput);
                backRight.setPower(rightStickInput);
                backLeft.setPower(rightStickInput);
            }
            if (rightStickInput < 0.1) {
                frontRight.setPower(rightStickInput);
                frontLeft.setPower(rightStickInput);
                backRight.setPower(rightStickInput);
                backLeft.setPower(rightStickInput);

            }
        }
    }
}

