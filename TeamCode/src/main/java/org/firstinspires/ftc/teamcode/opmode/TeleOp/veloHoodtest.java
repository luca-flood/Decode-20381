package org.firstinspires.ftc.teamcode.opmode.TeleOp;


import static dev.nextftc.bindings.Bindings.button;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.delays.Delay;
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

@TeleOp(name="testmaxxing", group="Linear Opmode")
public class veloHoodtest extends NextFTCOpMode {

    MotorEx turret = new MotorEx("turret").zeroed().reversed();
    private ControlSystem controller;
    public double getDistance(double ta) {
        return 75.16142 * Math.pow(ta, -0.47932);
    }
    public double getHoodAngle(double distance) {
        return (distance >= 120) ? -0.0210202 * distance +3.48553 : 0.00578881 * distance - 0.00802588;
    }

    public double getVelocity(double distance) {
        return (distance >= 120) ? 19.17589 * distance -582.51652 : 10.81866 * distance + 1084.95409;
    }
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

    public double getVel(double dist) {
        return 7.65268 * dist + 1123.06996;
    }

    public double getHood(double dist) {
        return 71.71201 * Math.pow(dist, -1.17652);
    }

    boolean running;
    double targetVel;
    double targetHood = 0;
    double yaw;
    double distance;
    double delay;
    double clankerX;
    double clankerY;
    double clankerR;
    double blueX = 0;
    double blueY = 144;
    double offset = 8.8;
    double goalX = 7.1;
    double goalY = 144;
    double heading = 90;
    double theta;
    double ticks;
    Follower clanka;


    MotorEx frontLeftMotor;
    MotorEx frontRightMotor;
    MotorEx backLeftMotor;
    MotorEx backRightMotor;
    Limelight3A limelight;
    double leftMax = 691;
    double rightMax = -809;

    @Override
    public void onInit() {
        clanka = PedroComponent.follower();
        transferSubsystem.INSTANCE.toNeutral.schedule();

        PedroComponent.follower().setStartingPose(new Pose(72, 72, Math.toRadians(90)));
        turret.setCurrentPosition(0);
        turret.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        controller = ControlSystem.builder()
                .posPid(0.016, 0.0, 0.0001)
                .basicFF(0, 0, 0.1)
                .build();
        controller.setGoal(new KineticState(0.0));


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
                whenBecomesTrue(multiFunctionSubsystem.INSTANCE.shootSeq()
                );

        Button gamepad1dpadright = button(() -> gamepad1.dpad_right);
        gamepad1dpadright
                .whenBecomesTrue(multiFunctionSubsystem.INSTANCE.transpherSequencNiga());

        Button gamepad1y = button(() -> gamepad1.y);
        gamepad1y.
                whenTrue(hoodSubsystem.INSTANCE.close);

        Button gamepad1a = button(() -> gamepad1.a);
        gamepad1a.
                whenBecomesTrue(hoodSubsystem.INSTANCE.lowest);

        driverControlled.schedule();
    }

    public void onUpdate() {
        theta = H2(calcDigger());
        ticks = tickAdjustment(calcDigger());
        controller.setGoal(new KineticState(-ticks));

        KineticState currentState = new KineticState(turret.getCurrentPosition(), turret.getVelocity());
        double turretPower = controller.calculate(currentState);
        turret.setPower(turretPower);

        heading = PedroComponent.follower().getHeading();
        clankerX = PedroComponent.follower().getPose().getX();
        clankerY = PedroComponent.follower().getPose().getY();

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
            delay = llresult.getStaleness();
            telemetry.update();
        }

        distance = Math.sqrt(Math.pow((blueX - clankerX), 2) + Math.pow((blueY - clankerY), 2)) + offset;

        targetVel = (running) ? getVel(distance) : 0;
        targetHood = (running) ? getHood(distance) : 0;

        outtakeSubsystem.INSTANCE.setVel(targetVel).schedule();
        hoodSubsystem.INSTANCE.goon(targetHood).schedule();

//        turretSubsystemLimelight.INSTANCE.updateAngle(yaw);


//        if (gamepad1.dpad_up) {
//            outtakeSubsystem.INSTANCE.closeMid().schedule();
//        }
//        else if (gamepad1.left_trigger > 0.1) {
//            outtakeSubsystem.INSTANCE.off().schedule();
//        }

        if (gamepad1.left_bumper) {
            intakeSubsystem.INSTANCE.spit.schedule();
        }
        else if (gamepad1.right_bumper) {
            intakeSubsystem.INSTANCE.eat.schedule();;
        }
        else if (gamepad1.b) {
            intakeSubsystem.INSTANCE.sleep.schedule();
        }

        if (gamepad1.right_trigger > 0.1) {
            running = true;
        }
        if (gamepad1.left_trigger > 0.1) {
            running = false;
        }

        if (gamepad1.a) {
            new SequentialGroup(
                    transferSubsystem.INSTANCE.toOuttake,
                    new Delay(0.5),
                    transferSubsystem.INSTANCE.toNeutral
//                      intakeSubsystem.INSTANCE.eat,
//                      new Delay(0.5),
//                      intakeSubsystem.INSTANCE.sleep
            ).schedule();
        }


//        telemetry.addData("Tx", llresult.getTx());
//        telemetry.addData("Ty", llresult.getTy());
//        telemetry.addData("Ta", llresult.getTa());
//
//        telemetry.addData("delay", llresult.getStaleness());
//
//
//        telemetry.addData("LL Latency", captureLatency + targetingLatency);
//        telemetry.addData("Parse Latency", parseLatency);
//
//
//        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
//                status.getTemp(), status.getCpu(),(int)status.getFps());
//
//
//        telemetry.addData("Yaw", yaw);
//        telemetry.addData("Found", tagFound);
//
//        // Show the power being commanded by the PID for debugging
//        telemetry.addData("Turret Power Command",
//                turretSubsystem.INSTANCE.controlSystem.calculate(
//                        // Use the subsystem's internal yaw value for debugging the PID calculation
//                        new KineticState(turretSubsystem.INSTANCE.currentYaw, 0.0, 0.0)));
        telemetry.addData("Target Velocity: ", targetVel);
//        telemetry.addData("Actual Velocity: ", outtake.getVelocity());
        telemetry.addData("Hood Position: ", hoodSubsystem.INSTANCE.getDaddy());
        telemetry.addData("Transfer Position: ", transferSubsystem.INSTANCE.getJawn());
        telemetry.addData("Current Velocity", outtakeSubsystem.INSTANCE.getJawn());
        telemetry.addData("Target Area", llresult.getTa());
        telemetry.addData("Distance (inches)", distance);
        telemetry.addData("Tag Found", tagFound);
        telemetry.addData("Optimal Hood Angle", getHood(distance));
        telemetry.addData("Optimal Velocity", getVel(distance));
        telemetry.addData("Bot X", clankerX);
        telemetry.addData("Bot Y", clankerY);

        telemetry.update();
    }

    public double normalize(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle <= -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    public double atan2(double y, double x) {
        return Math.atan2(y, x);
    }

    public double calcDigger() {
        return atan2(goalY - (7 + clankerY), goalX - clankerX);
    }

    public double H2(double digger) {
        return normalize(digger - heading) * 180 / Math.PI;
    }

    public double tickAdjustment(double digger) {
        double calcTicks = H2(digger) * (130.0 / 36.0) * (384.5 / 360.0);
        double ticksPerRev = 384.5 * (130.0 / 36.0);

        if (leftMax != 0 && calcTicks > leftMax) calcTicks -= ticksPerRev;
        else if (rightMax != 0 && calcTicks < rightMax) calcTicks += ticksPerRev;

        return calcTicks;
    }
}
