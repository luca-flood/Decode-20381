package org.firstinspires.ftc.teamcode.opmode.TeleOp.oldCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;



@Disabled
@TeleOp(name="clubRush", group="Linear Opmode")
public class clubRush extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("leftFront");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("leftRear");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("rightFront");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("rightRear");

        Servo axonRight = hardwareMap.get(Servo.class, "axonRight");
        Servo axonLeft = hardwareMap.get(Servo.class, "axonLeft");

        motorBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        DcMotor leftLift = hardwareMap.get(DcMotor.class, "leftLift");

        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setTargetPosition(0);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLift.setPower(0);
        leftLift.setDirection(DcMotorSimple.Direction.FORWARD);

        DcMotor rightLift = hardwareMap.get(DcMotor.class, "rightLift");

        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setTargetPosition(0);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setPower(0);
        rightLift.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        double Rpos = 0.7767;
        double Lpos = 0.2;

        int Rlift = -40;
        int Llift = -97;

        axonRight.setPosition(0.7767);
        axonLeft.setPosition(0.2);

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
                motorFrontLeft.setPower(frontLeftPower * 0.85);
                motorBackLeft.setPower(backLeftPower * 0.65);
                motorFrontRight.setPower(frontRightPower * 0.55);
                motorBackRight.setPower(backRightPower * 0.55);
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
                Rpos = 0.7767;
                axonRight.setPosition(Rpos);
                telemetry.addData("Axon Right Current Pos", axonRight.getPosition());
                telemetry.update();
            }

            if (gamepad1.b) {
                Rpos = 0.65;
                axonRight.setPosition(Rpos);
                telemetry.addData("Axon Right Current Pos", axonRight.getPosition());
                telemetry.update();
            }
            if (gamepad1.x) {
                Lpos = 0.2;
                axonLeft.setPosition(Lpos);
                telemetry.addData("Axon Left Current Pos", axonLeft.getPosition());
                telemetry.update();
            }
            if (gamepad1.y) {
                Lpos = 0.42;
                axonLeft.setPosition(Lpos);
                telemetry.addData("Axon Left Current Pos", axonLeft.getPosition());
                telemetry.update();
            }

            if (gamepad1.right_trigger > 0.1){
                Rlift += 10;
                if (Rlift > 8287){
                    Rlift = 8287;
                }
                if (Rlift < -40){
                    Rlift = -40;
                }
                rightLift.setPower(1);
                rightLift.setTargetPosition(Rlift);
                telemetry.addData("Right Lift Current Pos", rightLift.getCurrentPosition());
                telemetry.update();
            }

            if (gamepad1.right_bumper){
                Rlift -= 10;
                if (Rlift > 8287){
                    Rlift = 8287;
                }
                if (Rlift < -40){
                    Rlift = -40;
                }
                rightLift.setPower(1);
                rightLift.setTargetPosition(Rlift);
                telemetry.addData("Right Lift Current Pos", rightLift.getCurrentPosition());
                telemetry.update();
            }

            if (gamepad1.left_trigger > 0.1){
                Llift -= 10;
                if (Llift > -97){
                    Llift = -97;
                }
                if (Llift < -8149){
                    Llift = 8149;
                }
                leftLift.setPower(1);
                leftLift.setTargetPosition(Llift);
                telemetry.addData("Left Lift Current Pos", leftLift.getCurrentPosition());
                telemetry.update();
            }

            if (gamepad1.left_bumper){
                Llift += 10;
                if (Llift > -97){
                    Llift = -97;
                }
                if (Llift < -8149){
                    Llift = 8149;
                }
                leftLift.setPower(1);
                leftLift.setTargetPosition(Llift);
                telemetry.addData("Left Lift Current Pos", leftLift.getCurrentPosition());
                telemetry.update();
            }




        }
    }

    @Disabled

    @TeleOp(name = "67 Phonk")
    public static class Meet1DriverControl extends LinearOpMode {


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
}

