package org.firstinspires.ftc.teamcode.opmode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Disabled

@TeleOp(name="motor.test", group="Linear Opmode")
public class motorTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {


        DcMotor test = hardwareMap.get(DcMotor.class, "test");
        DcMotor test2 = hardwareMap.get(DcMotor.class, "test2");


        //leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        test.setTargetPosition(0);
        //leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // leftLift.setPower(0);
        test.setDirection(DcMotorSimple.Direction.FORWARD);

        int pos = 0;


        waitForStart();

        while (opModeIsActive()) {
/*
          if (gamepad1.x) {
                pos -= 10;
                //pos = Math.max(-7935, Math.min(-135, pos));
              leftLift.setTargetPosition(pos);
              leftLift.setPower(1);
              telemetry.addData("Current Pos", leftLift.getCurrentPosition());
              telemetry.update();
            }

            if (gamepad1.y) {
                pos += 10;
                //pos = Math.max(-7935, Math.min(-135, pos));
                leftLift.setTargetPosition(pos);
                leftLift.setPower(1);
                telemetry.addData("Current Pos", leftLift.getCurrentPosition());
                telemetry.update();
            }
            */

            if (gamepad1.a) {
                test.setPower(1);
            }
            if (gamepad1.b) {
                test.setPower(-1);
            }
            if (gamepad1.x) {
                test.setPower(0);
            }
            if (gamepad1.right_bumper) {
                test2.setPower(1);
            }
            if (gamepad1.left_bumper) {
                test2.setPower(-1);
            }
            if (gamepad1.y) {
                test2.setPower(0);
            }
        }
    }
}