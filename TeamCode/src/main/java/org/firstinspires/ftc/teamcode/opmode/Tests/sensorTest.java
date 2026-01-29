package org.firstinspires.ftc.teamcode.opmode.Tests;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.hoodSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.intakeSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.multiFunctionSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.outtakeSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.transferSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.turretSubsystemLimelight;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;


@Disabled

@TeleOp(name = "nigam", group = "yes")
public class sensorTest extends NextFTCOpMode {

    Rev2mDistanceSensor proxySens;
    RevColorSensorV3 colorSens;
    Servo backlight;
    boolean hasArtifact;

    public sensorTest() {

        addComponents(
                BindingsComponent.INSTANCE,
                new SubsystemComponent(
                        multiFunctionSubsystem.INSTANCE,
                        outtakeSubsystem.INSTANCE,
                        intakeSubsystem.INSTANCE,
                        transferSubsystem.INSTANCE,
                        hoodSubsystem.INSTANCE,
                        turretSubsystemLimelight.INSTANCE
                ),
                BulkReadComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)

        );
    }

    @Override
    public void onInit() {
        proxySens = hardwareMap.get(Rev2mDistanceSensor.class, "proxySensor");
        colorSens = hardwareMap.get(RevColorSensorV3.class, "color sensor");
        backlight = hardwareMap.get(Servo.class, "backlight");
    }

    @Override
    public void onUpdate() {
        if (proxySens.getDistance(DistanceUnit.INCH) < 5) {
            hasArtifact = true;
            backlight.setPosition(4.720);
        }
        else {
            hasArtifact = false;
            backlight.setPosition(0.277);
        }

        if (gamepad1.right_trigger > 0.1) {
            intakeSubsystem.INSTANCE.eat.schedule();
        }
        else if (gamepad1.left_trigger > 0.1) {
            intakeSubsystem.INSTANCE.spit.schedule();
        }
        else {
            intakeSubsystem.INSTANCE.sleep.schedule();
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