package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.outtakeSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.transferSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.turretSubsystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

import org.firstinspires.ftc.teamcode.opmode.Subsystems.hoodSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.intakeSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.multiFunctionSubsystem;

import dev.nextftc.ftc.Gamepads;
import dev.nextftc.hardware.driving.DriverControlledCommand;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;
@Disabled

@TeleOp(name="turretTestNextFTC", group=" NextFTC Opmode")
public class turretNextFTC extends NextFTCOpMode {
    public turretNextFTC() {

        addComponents(
                BindingsComponent.INSTANCE,
                new SubsystemComponent(
                        turretSubsystem.INSTANCE,
                        transferSubsystem.INSTANCE,
                        hoodSubsystem.INSTANCE,
                        intakeSubsystem.INSTANCE,
                        outtakeSubsystem.INSTANCE,
                        multiFunctionSubsystem.INSTANCE),
                BulkReadComponent.INSTANCE
        );
    }

    public double fx = 1385.92;
    public double fy = 1385.92;
    public double cx = 951.982;
    public double cy = 534.084;
    public boolean calibratedCamera;

    public AprilTagProcessor aprilTagProcessor;

    private final MotorEx frontLeftMotor = new MotorEx("leftFront");
    private final MotorEx frontRightMotor = new MotorEx("rightFront");
    private final MotorEx backLeftMotor = new MotorEx("leftRear");
    private final MotorEx backRightMotor = new MotorEx("rightRear");

    double yaw;

    double distance;

    double prevTime;

//    private double integral = 0;
//    private double lastError = 0;

    public void onInit() {
        if (calibratedCamera) {
            aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        } else {
            aprilTagProcessor = new AprilTagProcessor.Builder()
                    .setLensIntrinsics(fx, fy, cx, cy)
                    .build();
        }

        multiFunctionSubsystem.INSTANCE.stopPlease().schedule();
        transferSubsystem.INSTANCE.toNeutral.schedule();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Camera"))
                .addProcessor(aprilTagProcessor)

                .build();

        waitForStart();

        frontLeftMotor.setDirection(-1);
        backLeftMotor.setDirection(-1);
        frontRightMotor.setDirection(1);
        backRightMotor.setDirection(1);
    }

    @Override
    public void onStartButtonPressed() {

        DriverControlledCommand driverControlled = new MecanumDriverControlled(
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX()
       );





        driverControlled.schedule();

    }

    public void onUpdate() {
        BindingManager.update();

        boolean tagFound = false;

        for (AprilTagDetection detection : aprilTagProcessor.getDetections()) {
            int id = detection.id;
            // Target IDs 20 or 24 (assuming these are the depot tags)
            if (id == 20 || id == 24) {
                // Yaw is the orientation angle (in radians) of the tag relative to the camera
                yaw = detection.ftcPose.yaw;
                distance = detection.ftcPose.range;
                tagFound = true;
                // We break after finding one target to avoid potential conflicts if multiple tags are visible
                break;
            }
        }

        if (tagFound) {
            turretSubsystem.INSTANCE.updateAngle(yaw);
        }

        //turretSubsystem.INSTANCE.updateAngle(yaw);

        telemetry.addData("Tag Found", tagFound ? "YES" : "NO");
        telemetry.addData("Turret Yaw Error (rad)", String.format("%.4f", yaw));
        telemetry.addData("Distance (in)", String.format("%.2f", distance));
        telemetry.addData("Chilling", turretSubsystem.INSTANCE.chill);

        // Show the power being commanded by the PID for debugging
        telemetry.addData("Turret Power Command",
                turretSubsystem.INSTANCE.controlSystem.calculate(
                        // Use the subsystem's internal yaw value for debugging the PID calculation
                        new KineticState(turretSubsystem.INSTANCE.currentYaw, 0.0, 0.0)));

        telemetry.update();

    }

    @Override
    public void onStop() {
        BindingManager.reset();
        super.onStop();
    }
}
