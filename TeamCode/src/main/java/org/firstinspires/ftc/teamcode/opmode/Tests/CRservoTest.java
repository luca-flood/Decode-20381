package org.firstinspires.ftc.teamcode.opmode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImplEx;

@Disabled

@TeleOp(name="turretTest", group="Linear Opmode")

public class CRservoTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        CRServoImplEx turretServo = hardwareMap.get(CRServoImplEx.class, "turretServo");
       // Servo LeftClawServo = hardwareMap.get(Servo.class, "LeftClawServo");

        waitForStart();



        while (opModeIsActive()) {


            if (gamepad1.b){
                turretServo.setPower(1);
            }
            else if (gamepad1.a){
                turretServo.setPower(-1);

            }
            else{
                turretServo.setPower(0);
            }


        }

    }
}