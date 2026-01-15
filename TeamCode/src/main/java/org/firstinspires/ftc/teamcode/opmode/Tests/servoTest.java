package org.firstinspires.ftc.teamcode.opmode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;





@Disabled

@TeleOp(name="servo.test", group="Linear Opmode")
public class servoTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Servo axonLeft = hardwareMap.get(Servo.class, "axonLeft");
        // CRServo ClawWristServo = hardwareMap.get(CRServo.class, "ClawWristServo");

        /*DcMotor liftRotateMotorRight = hardwareMap.get(DcMotor.class, "liftRotateMotorRight");

        liftRotateMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRotateMotorRight.setTargetPosition(0);
        liftRotateMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRotateMotorRight.setPower(0);
*/
        double pos = 0.2;

        axonLeft.setPosition(0.2);

        waitForStart();

        while (opModeIsActive()) {

          /*  if (gamepad1.a) {
                axonMini.setPosition(1);
            }

            if (gamepad1.b) {
                axonMini.setPosition(0);
            }
            if (gamepad1.x) {
                axonMini.setPosition(0.5);
            }
*/
            if (gamepad1.a){
                pos += 0.001;
                if (pos > 0.42) {
                    pos = 0.42;
                }
                axonLeft.setPosition(pos);
                telemetry.addData("Current Pos", axonLeft.getPosition());
                telemetry.update();
            }

            if (gamepad1.b) {
                pos -= 0.001;
                if (pos < 0.2) {
                    pos = 0.2;
                }
                axonLeft.setPosition(pos);
                telemetry.addData("Current Pos", axonLeft.getPosition());
                telemetry.update();
            }


  /*          if (gamepad1.x) {
                pos -= 10;
                pos = Math.max(-7935, Math.min(-135, pos));
                liftRotateMotorRight.setTargetPosition(pos);
                liftRotateMotorRight.setPower(1);
            }

            if (gamepad1.y) {
                pos += 10;
                pos = Math.max(-7935, Math.min(-135, pos));
                liftRotateMotorRight.setTargetPosition(pos);
                liftRotateMotorRight.setPower(1);
            }

                if (gamepad1.right_trigger >= 0.1) {
                    liftRotateMotorRight.setPower(0.2);
                }

            if (gamepad1.left_trigger >= 0.1) {
                liftRotateMotorRight.setPower(-0.2);
            }*/


        }

    }
}
