package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled

@TeleOp(name = "67 Phonk")
public class Meet1DriverControl extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        DcMotorEx outtake = hardwareMap.get(DcMotorEx.class, "outtake");

        DcMotor fL = hardwareMap.dcMotor.get("leftFront");
        DcMotor bL = hardwareMap.dcMotor.get("leftRear");
        DcMotor fR = hardwareMap.dcMotor.get("rightFront");
        DcMotor bR = hardwareMap.dcMotor.get("rightRear");

        fR.setDirection(DcMotorSimple.Direction.REVERSE);
        bR.setDirection(DcMotorSimple.Direction.REVERSE);

        Servo flap = hardwareMap.get(Servo.class, "flap");


        waitForStart();

        double oVelocity = 0;
        double currentPos = 0;


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
//trigger


// abxy


            //outtake
            if (gamepad1.right_bumper){
                outtake.setVelocity(oVelocity);
            }
            else if (gamepad1.left_bumper){
                outtake.setPower(1);
            }
            else if (gamepad2.right_bumper){
                outtake.setVelocity(oVelocity);
            }
            else if (gamepad2.left_bumper){
                outtake.setPower(-1);
            }
            else {
                outtake.setVelocity(0);
            }

            //manual hood
            if (gamepad1.dpad_up){
                currentPos = flap.getPosition();
                flap.setPosition(currentPos + 0.01);
            }
            if (gamepad1.dpad_down){
                currentPos = flap.getPosition();
                flap.setPosition(currentPos - 0.01);
            }

// gamepad 2


            if (gamepad1.left_trigger > 0.1) {
                intake.setPower(1);
            }
            else if (gamepad1.right_trigger > 0.1) {
                intake.setPower(-1);
            }
            else if (gamepad2.right_trigger>=0.1) {
                intake.setPower(-1);
            }
            else if (gamepad2.left_trigger>=0.1) {
                intake.setPower(1);
            }
            else {
                intake.setPower(0);
            }


            if (gamepad1.a) {
                flap.setPosition(0.975);
                oVelocity = -1400;
                // close

            }
            if (gamepad1.b) {
                flap.setPosition(0.605);
                oVelocity = -1600;
                //close mid
            }
            if (gamepad1.x) {
                flap.setPosition(0.4867);
                oVelocity = -1900;
                //mid

            }
            if (gamepad1.y) {
                flap.setPosition(0.5456);
                oVelocity = -2200;
                //far
            }

            if (gamepad2.a){
                flap.setPosition(0.975);
                oVelocity = -1400;
                // close
            }

            if (gamepad2.x) {
                flap.setPosition(0.975);
                oVelocity = -1400;
                // close

            }
            if (gamepad2.b) {
                flap.setPosition(0.605);
                oVelocity = -1600;
                //close mid
            }
            if (gamepad2.x) {
                flap.setPosition(0.4867);
                oVelocity = -1900;
                //mid

            }
            if (gamepad2.y) {
                flap.setPosition(0.5456);
                oVelocity = -2200;
                //far
            }


            if (gamepad2.dpad_up){
                currentPos = flap.getPosition();
                flap.setPosition(currentPos + 0.01);
            }
            if (gamepad2.dpad_down){
                currentPos = flap.getPosition();
                flap.setPosition(currentPos - 0.01);
            }
            if (gamepad2.dpad_left || gamepad1.dpad_left){
                telemetry.addLine("Daniel you have no rights");
            }



            telemetry.addData("Current Velocity", outtake.getVelocity());
            telemetry.addData("Current Position :P", flap.getPosition());
            telemetry.addLine("close: (a) Velocity: 1400");
            telemetry.addLine("closeMid: (b) Velocity: 1600");
            telemetry.addLine("mid: (x) Velocity: 1900" );
            telemetry.addLine("far: (y) Velocity: 2100");
            if (outtake.getVelocity() >= oVelocity) {
                telemetry.addLine("Ready!!!");
            }
            else {
                telemetry.addLine("Keep Gooning");
            }
            telemetry.update();

        }
    }
}