package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.hardware.impl.CRServoEx;
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