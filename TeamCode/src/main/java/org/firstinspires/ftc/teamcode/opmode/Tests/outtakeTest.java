package org.firstinspires.ftc.teamcode.opmode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;




@Disabled

@TeleOp(name="outtakeTest", group="Linear Opmode")

public class outtakeTest extends LinearOpMode  {


    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        DcMotor transfer = hardwareMap.get(DcMotor.class, "transfer");
        DcMotor outtake = hardwareMap.get(DcMotor.class, "outtake");

        DcMotor fL = hardwareMap.dcMotor.get("leftFront");
        DcMotor bL = hardwareMap.dcMotor.get("leftRear");
        DcMotor fR = hardwareMap.dcMotor.get("rightFront");
        DcMotor bR = hardwareMap.dcMotor.get("rightRear");

        fR.setDirection(DcMotorSimple.Direction.REVERSE);
        bR.setDirection(DcMotorSimple.Direction.REVERSE);

        Servo flap = hardwareMap.get(Servo.class, "flap");


        waitForStart();


        while (opModeIsActive()){

            double y = gamepad1.left_stick_y;
            double rx = -gamepad1.right_stick_x * 1.1;
            double x = -gamepad1.left_stick_x;

            double y2 = gamepad2.left_stick_y;
            double rx2 = -gamepad2.right_stick_x * 1.1;
            double x2 = -gamepad2.left_stick_x;

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
                fL.setPower(frontLeftPower * 0.85);
                bL.setPower(backLeftPower * 0.85);
                fR.setPower(frontRightPower * 0.85);
                bR.setPower(backRightPower * 0.85);
            } else if (gamepad2.left_stick_y >= 0.1 || gamepad2.left_stick_x >= 0.1 || gamepad2.left_stick_y <= -0.1 || gamepad2.left_stick_x <= -0.1 || gamepad2.right_stick_y >= 0.1 || gamepad2.right_stick_x >= 0.1 || gamepad2.right_stick_y <= -0.1 || gamepad2.right_stick_x <= -0.1) {
                fL.setPower(frontLeftPower2 * 0.3);
                bL.setPower(backLeftPower2 * 0.3);
                fR.setPower(frontRightPower2 * 0.3);
                bR.setPower(backRightPower2 * 0.3);
            } else {
                fL.setPower(0);
                bL.setPower(0);
                fR.setPower(0);
                bR.setPower(0);
            }

            if (gamepad1.a){
                outtake.setPower(1);
            }
            if (gamepad1.b){
                outtake.setPower(-1);
            }
            if (gamepad1.x){
                outtake.setPower(0);
            }

            if (gamepad1.right_bumper){
                intake.setPower(1);
            }
            if (gamepad1.right_bumper){
                intake.setPower(-1);
            }
            if (gamepad1.y){
                intake.setPower(0);
            }


        }
    }
}
