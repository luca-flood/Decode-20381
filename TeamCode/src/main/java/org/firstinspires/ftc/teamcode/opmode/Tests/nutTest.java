package org.firstinspires.ftc.teamcode.opmode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;



@Disabled

@TeleOp(name="intake nut", group="Linear Opmode")
public class nutTest extends LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {

        DcMotorEx intake = (DcMotorEx) hardwareMap.dcMotor.get("intake");
        DcMotorEx outtake = (DcMotorEx) hardwareMap.dcMotor.get("outtake");

        Servo transfer = hardwareMap.servo.get("transfer");
        Servo hood = hardwareMap.servo.get("flap");


        DcMotor fL = hardwareMap.dcMotor.get("leftFront");
        DcMotor bL = hardwareMap.dcMotor.get("leftRear");
        DcMotor fR = hardwareMap.dcMotor.get("rightFront");
        DcMotor bR = hardwareMap.dcMotor.get("rightRear");

        fR.setDirection(DcMotorSimple.Direction.REVERSE);
        bR.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();

        double transferPos = 0.5;
        double outtakeVelocity = 0;
        double hoodPos = 1;
        //boolean ifPressed = false;

        while (opModeIsActive()) {

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


            if (gamepad1.y){
                //transferPos = transferPos + 0.0001;
                telemetry.addData("transfer position", transfer.getPosition());
                telemetry.update();
                transfer.setPosition(0.89);
            }
            else{
                transfer.setPosition(0.7048);

            }

            if (gamepad1.x){
                //transferPos = transferPos - 0.0001;
                transfer.setPosition(0.7048);
                telemetry.addData("transfer position", transfer.getPosition());
                telemetry.update();

            }

            if (gamepad1.right_bumper){
                intake.setPower(1);
            }
            if (gamepad1.left_bumper){
                intake.setPower(-1);
            }
            else {
                intake.setPower(0);
            }
            if (gamepad1.a){
                outtakeVelocity = outtakeVelocity - 1;

            }
            if (gamepad1.right_trigger > 0.1){
                outtake.setVelocity(outtakeVelocity);

            }
            if (gamepad1.left_trigger > 0.1){
                outtake.setVelocity(0);
                //outtakeVelocity = outtakeVelocity + 10;
            }
            else {
                outtake.setPower(0);
            }

            if (gamepad1.a){
                hood.setPosition(hoodPos);
                hoodPos += 0.001;
            }
            if (gamepad1.b){
                hood.setPosition(hoodPos);
                hoodPos -= 0.001;
            }


            telemetry.addData("outtake velocity",outtake.getVelocity());
            telemetry.addData("hood pos", hood.getPosition());
            telemetry.update();
        }
    }



}
