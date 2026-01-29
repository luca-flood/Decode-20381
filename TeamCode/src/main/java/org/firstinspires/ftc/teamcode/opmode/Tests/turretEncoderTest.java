package org.firstinspires.ftc.teamcode.opmode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;



@Disabled

@TeleOp(name="turret encoder test", group="walahi bismallah")

public class turretEncoderTest extends LinearOpMode {

    double current = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor turret = hardwareMap.get(DcMotor.class, "turret");

        waitForStart();

        current = turret.getCurrentPosition();



        while (opModeIsActive()) {

            current = turret.getCurrentPosition();


            telemetry.addData("turret value", current);

            if(gamepad1.a){
                turret.setPower(-0.5);
            }
            else if (gamepad1.b){
                turret.setPower(0.5);
            }
            else{
                turret.setPower(0);
            }

            telemetry.update();
        }
        }
    }