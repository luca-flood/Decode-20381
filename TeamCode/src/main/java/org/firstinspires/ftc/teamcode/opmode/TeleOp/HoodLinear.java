package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Hood Test", group="Linear Opmode")
public class HoodLinear extends LinearOpMode {
    double position = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo hood = hardwareMap.servo.get("hood");

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.left_bumper) {
                position += 0.01;
            }
            else if (gamepad1.right_bumper) {
                position -= 0.01;
            }
            //hood.setPosition(position);
            if(gamepad1.a){
                hood.setPosition(0.2);
            }
            else if(gamepad1.b){
                hood.setPosition(0.7);
            }
            telemetry.addData("Hood Position - Diddy", hood.getPosition());
            telemetry.update();
        }
    }
}
