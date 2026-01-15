package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled

@TeleOp(name="tiecheDec23", group="Linear Opmode")

public class tiecheDec23 extends LinearOpMode {

    public void runOpMode() throws InterruptedException{


        DcMotorEx outtakeTop = hardwareMap.get(DcMotorEx.class, "outtake1");
        DcMotorEx outtakeBottom = hardwareMap.get(DcMotorEx.class, "outtake1");

        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("leftFront");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("leftRear");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("rightFront");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("rightRear");

        DcMotor intake = hardwareMap.dcMotor.get("intake");

        Servo transfer = hardwareMap.servo.get("transfer");
        Servo hood = hardwareMap.servo.get("transfer");

        waitForStart();

        hood.setPosition(0);
        transfer.setPosition(0.68);

        double hoodPos = 0;


        while (opModeIsActive()) {

            double y = gamepad1.left_stick_y;
            double rx = -gamepad1.right_stick_x * 1.1;
            double x = -gamepad1.left_stick_x;

// Stupid Code
            double y2 = gamepad2.left_stick_y;
            double rx2 = -gamepad2.right_stick_x * 1.1;
            double x2 = -gamepad2.left_stick_x;

            telemetry.addLine("left stick y" + y);
            telemetry.addLine("left stick x" + x);
            telemetry.addLine("right stick x" + rx);

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            double denominator2 = Math.max(Math.abs(y2) + Math.abs(x2) + Math.abs(rx2), 1);
            double frontLeftPower2 = (y2 + x2 + rx2) / denominator2;
            double backLeftPower2 = (y2 - x2 + rx2) / denominator2;
            double frontRightPower2 = (y2 - x2 - rx2) / denominator2;
            double backRightPower2 = (y2 + x2 - rx2) / denominator2;


            if (gamepad1.left_stick_y >= 0.1 || gamepad1.left_stick_x >= 0.1 || gamepad1.left_stick_y <= -0.1 || gamepad1.left_stick_x <= -0.1 || gamepad1.right_stick_y >= 0.2 || gamepad1.right_stick_x >= 0.2 || gamepad1.right_stick_y <= -0.2 || gamepad1.right_stick_x <= -0.2) {
                motorFrontLeft.setPower(frontLeftPower * 0.85);
                motorBackLeft.setPower(backLeftPower * 0.85);
                motorFrontRight.setPower(frontRightPower * 0.85);
                motorBackRight.setPower(backRightPower * 0.85);
            } else if (gamepad2.left_stick_y >= 0.1 || gamepad2.left_stick_x >= 0.1 || gamepad2.left_stick_y <= -0.1 || gamepad2.left_stick_x <= -0.1 || gamepad2.right_stick_y >= 0.1 || gamepad2.right_stick_x >= 0.1 || gamepad2.right_stick_y <= -0.1 || gamepad2.right_stick_x <= -0.1) {
                motorFrontLeft.setPower(frontLeftPower2 * 0.3);
                motorBackLeft.setPower(backLeftPower2 * 0.3);
                motorFrontRight.setPower(frontRightPower2 * 0.3);
                motorBackRight.setPower(backRightPower2 * 0.3);
            } else {
                motorFrontLeft.setPower(0);
                motorBackLeft.setPower(0);
                motorFrontRight.setPower(0);
                motorBackRight.setPower(0);
            }



            if (gamepad1.a){
                transfer.setPosition(0.9);
            }
            else{
                transfer.setPosition(0.68);
            }


            if (gamepad1.right_trigger > 0.1){
                outtakeTop.setPower(1);
                outtakeBottom.setPower(1);
            }
            else {
                outtakeTop.setPower(0);
                outtakeBottom.setPower(0);
            }

            if (gamepad1.right_bumper){
                intake.setPower(1);
            }
            else {
                intake.setPower(0);
            }

            if (gamepad1.x){
                hoodPos += 0.01;
                hood.setPosition(hoodPos);
            }
            else if(gamepad1.y){
                hoodPos -= 0.01;
                hood.setPosition(hoodPos);
            }

            telemetry.addData("hood position", hood.getPosition());
            telemetry.addData( "top motor velocity", outtakeTop.getVelocity());
            telemetry.addData("bottom motor velocity", outtakeBottom.getVelocity());
            telemetry.addData("transfer position", transfer.getPosition());
            telemetry.update();

        }
    }
}
