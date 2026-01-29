package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.hoodSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.intakeSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.multiFunctionSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.outtakeSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.transferSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.turretSubsystem;
import org.firstinspires.ftc.teamcode.opmode.TeleOp.oldCode.NEXTftcOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;


@Disabled

@TeleOp(name = "I goon to femboys")
public class limelightLocalization extends NEXTftcOpMode {

    MotorEx frontLeftMotor;
    MotorEx frontRightMotor;
    MotorEx backLeftMotor;
    MotorEx backRightMotor;
    Limelight3A limelight;
    Follower clanka;
    Pose3D pose;

    public limelightLocalization() {
        addComponents(
                BindingsComponent.INSTANCE,
                new SubsystemComponent(
                        multiFunctionSubsystem.INSTANCE,
                        outtakeSubsystem.INSTANCE,
                        intakeSubsystem.INSTANCE,
                        transferSubsystem.INSTANCE,
                        hoodSubsystem.INSTANCE,
                        turretSubsystem.INSTANCE // used for kinematic turret logic
                ),
                BulkReadComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }

    @Override
    public void onInit() {
        frontLeftMotor = new MotorEx("leftFront");
        frontRightMotor = new MotorEx("rightFront");
        backLeftMotor = new MotorEx("leftRear");
        backRightMotor = new MotorEx("rightRear");

        frontRightMotor.setDirection(1);
        backRightMotor.setDirection(1);
        frontLeftMotor.setDirection(1);
        backLeftMotor.setDirection(1);

        clanka = PedroComponent.follower();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(3);
        telemetry.setMsTransmissionInterval(2);
    }

    @Override
    public void onStartButtonPressed() {
        limelight.start();

        new MecanumDriverControlled(
                frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor,
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX()
        ).schedule();

        clanka.setStartingPose(new Pose(72, 72, 90));
    }

    @Override
    public void onUpdate() {
        clanka.update();
        BindingManager.update();

        LLResult llresult = limelight.getLatestResult();
        boolean tagFound = (llresult != null && llresult.isValid());
        if (tagFound)
        {
            pose = llresult.getBotpose_MT2();
            telemetry.addData("MT2 X", pose.getPosition().x * 39.37);
            telemetry.addData("MT2 Y", pose.getPosition().y * 39.37);
            telemetry.addData("MT2 Z", pose.getPosition().z);
        }

        telemetry.addData("Tag Found", tagFound);
        telemetry.addData("Follower X", clanka.getPose().getX());
        telemetry.addData("Follower Y", clanka.getPose().getY());
        telemetry.addData("Follower Heading", clanka.getPose().getHeading());
        telemetry.update();
    }

}
