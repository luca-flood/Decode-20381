package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmode.Subsystems.outtakeSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.transferSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.turretSubsystem;

import com.qualcomm.hardware.limelightvision.LLStatus;

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

@TeleOp(name="limelightTurret", group=" NextFTC Opmode")
public class turretLimelight extends NextFTCOpMode {
    public turretLimelight() {

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


    private final MotorEx frontLeftMotor = new MotorEx("leftFront");
    private final MotorEx frontRightMotor = new MotorEx("rightFront");
    private final MotorEx backLeftMotor = new MotorEx("leftRear");
    private final MotorEx backRightMotor = new MotorEx("rightRear");

    Limelight3A limelight;

    double yaw;

    double distance;

    double delay;

    double prevTime;

//    private double integral = 0;
//    private double lastError = 0;

    public void onInit() {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);

        telemetry.setMsTransmissionInterval(2);

        waitForStart();

        frontLeftMotor.setDirection(-1);
        backLeftMotor.setDirection(-1);
        frontRightMotor.setDirection(1);
        backRightMotor.setDirection(1);
    }

    @Override
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

            //turretSubsystem.INSTANCE.updateAngle(llresult.getTx());
            yaw = llresult.getTx();
            distance = llresult.getTa();
            delay = llresult.getStaleness();
            telemetry.update();

        }
//        else if(!tagFound) {
//            turretSubsystem.INSTANCE.updateAngleCoast(yaw);
//        }


        turretSubsystem.INSTANCE.updateAngle(yaw);


        telemetry.addData("Tx", llresult.getTx());
        telemetry.addData("Ty", llresult.getTy());
        telemetry.addData("Ta", llresult.getTa());

        telemetry.addData("delay", llresult.getStaleness());


        telemetry.addData("LL Latency", captureLatency + targetingLatency);
        telemetry.addData("Parse Latency", parseLatency);


        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(),(int)status.getFps());


        telemetry.addData("Chilling", turretSubsystem.INSTANCE.chill);
        telemetry.addData("Yaw", yaw);

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
