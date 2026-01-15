package org.firstinspires.ftc.teamcode.opmode.TeleOp;


import static dev.nextftc.bindings.Bindings.button;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

import org.firstinspires.ftc.teamcode.opmode.Subsystems.hoodSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.intakeSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.multiFunctionSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.outtakeSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.transferSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.turretSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.turretSubsystemLimelight;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Disabled

@TeleOp(name="testmaxxing", group="Linear Opmode")
public class veloHoodtest extends NextFTCOpMode {
    public veloHoodtest() {

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

    double targetVel;
    double yaw;
    double distance;
    double delay;
    double clankerX;
    double clankerY;
    double clankerR;

    Follower clanka;


    MotorEx frontLeftMotor;
    MotorEx frontRightMotor;
    MotorEx backLeftMotor;
    MotorEx backRightMotor;
    Limelight3A limelight;

    @Override
    public void onInit() {
        clanka = PedroComponent.follower();

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

        targetVel = 0;

    }

    public void onStartButtonPressed() {
        limelight.start();

        DriverControlledCommand driverControlled = new MecanumDriverControlled(
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX()
        );

        Button gamepad1x = button(() -> gamepad1.x);
        gamepad1x.
                whenBecomesTrue(new SequentialGroup(
                        outtakeSubsystem.INSTANCE.closeMid(),
                        multiFunctionSubsystem.INSTANCE.transferSequenceM3CloseMid(),
                        outtakeSubsystem.INSTANCE.off()
                ));

        Button gamepad1dpadleft = button(() -> gamepad1.dpad_left);
        gamepad1dpadleft.
                whenBecomesTrue(multiFunctionSubsystem.INSTANCE.transpherSequencNiga()
                );

        Button gamepad1dpadright = button(() -> gamepad1.dpad_right);
        gamepad1dpadright.
                whenTrue(transferSubsystem.INSTANCE.toOuttake)
                .whenBecomesFalse(transferSubsystem.INSTANCE.toNeutral);

        Button gamepad1y = button(() -> gamepad1.y);
        gamepad1y.
                whenTrue(hoodSubsystem.INSTANCE.close);

        Button gamepad1a = button(() -> gamepad1.a);
        gamepad1a.
                whenBecomesTrue(hoodSubsystem.INSTANCE.lowest);

        Button gamepad1b = button(() -> gamepad1.b);
        gamepad1b.
                whenBecomesTrue(hoodSubsystem.INSTANCE.closeMid);

        driverControlled.schedule();
    }

    public void onUpdate() {

        clankerX = clanka.getPose().getX();
        clankerY = clanka.getPose().getY();
        clankerR = clanka.getHeading();

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

        turretSubsystemLimelight.INSTANCE.updateAngle(yaw);


        if (gamepad1.right_trigger > 0.1) {
            outtakeSubsystem.INSTANCE.closeMid().schedule();
        }
        else if (gamepad1.left_trigger > 0.1) {
            outtakeSubsystem.INSTANCE.off().schedule();
        }

//        if (gamepad1.left_bumper) {
//            intakeSubsystem.INSTANCE.spit.schedule();
//        }
//        else if (gamepad1.right_bumper) {
//            intakeSubsystem.INSTANCE.eat.schedule();;
//        }
//        else {
//            intakeSubsystem.INSTANCE.sleep.schedule();
//        }

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
        telemetry.addData("Target Velocity: ", targetVel);
//        telemetry.addData("Actual Velocity: ", outtake.getVelocity());
        telemetry.addData("Hood Position: ", hoodSubsystem.INSTANCE.getDaddy());
//        telemetry.addData("Transfer Position: ", transfer.getPosition());
        telemetry.addData("Current Velocity", outtakeSubsystem.INSTANCE.getJawn());

        telemetry.update();
    }
}
