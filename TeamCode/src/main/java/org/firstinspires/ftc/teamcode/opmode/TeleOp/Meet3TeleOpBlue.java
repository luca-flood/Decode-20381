package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static dev.nextftc.bindings.Bindings.button;

import org.firstinspires.ftc.teamcode.opmode.Subsystems.hoodSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.intakeSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.multiFunctionSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.outtakeSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.transferSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.turretSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.turretSubsystemLimelight;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.DriverControlledCommand;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;
@Disabled

@TeleOp(name="M3Blue", group="Linear Opmode")

public class Meet3TeleOpBlue extends NextFTCOpMode {

    public Meet3TeleOpBlue() {

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

    MotorEx frontLeftMotor;
    MotorEx frontRightMotor;
    MotorEx backLeftMotor;
    MotorEx backRightMotor;
    Limelight3A limelight;
    DriverControlledCommand driverControlled;
    double yaw;
    double distance;
    double delay;
    boolean tracking = false;

    @Override
    public void onInit() {
        PedroComponent.follower().setStartingPose(new Pose(72, 72, 90));


        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);

        telemetry.setMsTransmissionInterval(2);

        frontLeftMotor = new MotorEx("leftFront");
        frontRightMotor = new MotorEx("rightFront");
        backLeftMotor = new MotorEx("leftRear");
        backRightMotor = new MotorEx("rightRear");

        frontRightMotor.setDirection(1);
        backRightMotor.setDirection(1);
        frontLeftMotor.setDirection(1);
        backLeftMotor.setDirection(1);

    }

    public void onStartButtonPressed() {
        limelight.start();

        driverControlled = new MecanumDriverControlled(
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX()
        );

        driverControlled.setScalar(0.9);

        Button gamepad1a = button(() -> gamepad1.a);

        gamepad1a.
                whenBecomesTrue(
                        new SequentialGroup(
                            outtakeSubsystem.INSTANCE.closeMid(),
                            multiFunctionSubsystem.INSTANCE.transferSequenceM3()
                        ));

        Gamepads.gamepad1().leftTrigger().atLeast(0.1).whenTrue(
                outtakeSubsystem.INSTANCE.off()
        );

        Gamepads.gamepad1().rightTrigger().atLeast(0.1).whenTrue(
                new SequentialGroup(
                        hoodSubsystem.INSTANCE.closeMid,
                        outtakeSubsystem.INSTANCE.closeMid()
                )

        );

        Button gamepad1lb = button(() -> gamepad1.left_bumper);
        gamepad1lb
                .whenTrue(
                    intakeSubsystem.INSTANCE.spit
                )
                .whenFalse(
                    intakeSubsystem.INSTANCE.sleep
                );

        Button gamepad1rb = button(() -> gamepad1.right_bumper);
        gamepad1rb
                .whenTrue(
                        intakeSubsystem.INSTANCE.eat
                )
                .whenFalse(
                        intakeSubsystem.INSTANCE.sleep
                );

        Button gamepad1y = button(() -> gamepad1.y);
        gamepad1y
                .whenBecomesTrue(
                        multiFunctionSubsystem.INSTANCE.transpherSequencNiga()
                );

        Button gamepad1x = button(() -> gamepad1.x);
        gamepad1x
                .whenTrue(
                        turretSubsystemLimelight.INSTANCE.setPower(-1)
                )
                .whenFalse(
                        turretSubsystemLimelight.INSTANCE.setPower(0)
                );

        Button gamepad1b = button(() -> gamepad1.b);
        gamepad1b
                .whenTrue(
                        turretSubsystemLimelight.INSTANCE.setPower(1)
                )
                .whenFalse(
                        turretSubsystemLimelight.INSTANCE.setPower(0)
                );

        driverControlled.schedule();
    }

    public void onUpdate() {
        BindingManager.update();

        LLResult llresult = limelight.getLatestResult();
        LLStatus status = limelight.getStatus();

        double captureLatency = llresult.getCaptureLatency();
        double targetingLatency = llresult.getTargetingLatency();
        double parseLatency = llresult.getParseLatency();

        boolean tagFound = (llresult != null && llresult.isValid());


        if (tagFound){
            yaw = llresult.getTx();
            distance = llresult.getTa();
            delay = llresult.getStaleness();
            telemetry.update();

        }

        if (gamepad1.dpad_down) {
            tracking = !tracking;
        }

        if (tracking) {
            turretSubsystemLimelight.INSTANCE.updateAngle(yaw);
        }
        else {
            turretSubsystemLimelight.INSTANCE.setPower(0);
        }

        turretSubsystemLimelight.INSTANCE.updateAngle(yaw);

        if (gamepad1.dpad_left) {
            driverControlled.setScalar(0.3);
        }

        if (gamepad1.dpad_right) {
            driverControlled.setScalar(0.9);
        }


        telemetry.addData("Tx", llresult.getTx());
        telemetry.addData("Ty", llresult.getTy());
        telemetry.addData("Ta", llresult.getTa());

        telemetry.addData("delay", llresult.getStaleness());


        telemetry.addData("LL Latency", captureLatency + targetingLatency);
        telemetry.addData("Parse Latency", parseLatency);


        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(),(int)status.getFps());


        telemetry.addData("Yaw", yaw);
        telemetry.addData("Found", tagFound);

        // Show the power being commanded by the PID for debugging
        telemetry.addData("Turret Power Command",
                turretSubsystem.INSTANCE.controlSystem.calculate(
                        // Use the subsystem's internal yaw value for debugging the PID calculation
                        new KineticState(turretSubsystem.INSTANCE.currentYaw, 0.0, 0.0)));

        telemetry.addData("Hood Position: ", hoodSubsystem.INSTANCE.getDaddy());
//        telemetry.addData("Transfer Position: ", transfer.getPosition());
        telemetry.addData("Current Velocity", outtakeSubsystem.INSTANCE.getJawn());

        telemetry.update();
    }
}
