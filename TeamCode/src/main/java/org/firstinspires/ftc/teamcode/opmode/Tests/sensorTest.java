package org.firstinspires.ftc.teamcode.opmode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "color blob", group = "yes")
public class sensorTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        DistanceSensor proxySens = hardwareMap.get(DistanceSensor.class, "proxySens");
        NormalizedColorSensor colorSens = hardwareMap.get(NormalizedColorSensor.class, "colorSens");
        Servo backlight = hardwareMap.get(Servo.class, "backlight");

        boolean hasArtifact;

        waitForStart();
        while (opModeIsActive()) {
            if (proxySens.getDistance(DistanceUnit.CM) < 5) {
                hasArtifact = true;
                backlight.setPosition(4.720);
            }
            else {
                hasArtifact = false;
                backlight.setPosition(0.277);
            }

            telemetry.addData("Has Artifact: ", String.valueOf(hasArtifact));
            telemetry.addData("Light Detected: ", ((OpticalDistanceSensor) colorSens).getLightDetected());
            NormalizedRGBA colors = colorSens.getNormalizedColors();
            telemetry.addData("Red: ", "%.3f", colors.red);
            telemetry.addData("Green: ", "%.3f", colors.green);
            telemetry.addData("Blue: ", "%.3f", colors.blue);
            telemetry.update();
        }

    }
}
