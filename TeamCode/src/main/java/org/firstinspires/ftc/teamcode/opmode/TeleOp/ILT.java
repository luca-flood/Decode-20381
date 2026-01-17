package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import static dev.nextftc.bindings.Bindings.button;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
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

@TeleOp(name="ILT", group="Linear Opmode")
public class ILT extends NextFTCOpMode {

    // --- Hardware & Controllers ---
    MotorEx turret = new MotorEx("turret").zeroed().reversed();
    private ControlSystem controller;

    MotorEx frontLeftMotor;
    MotorEx frontRightMotor;
    MotorEx backLeftMotor;
    MotorEx backRightMotor;

    Limelight3A limelight;
    Follower clanka;



    // turret vars
    double goalX = 5;
    double goalY = 144;
    double blueX = 0;
    double blueY = 144;
    double offset = 8.8;

    double clankerX, clankerY, clankerR, heading;
    double theta, ticks;
    double leftMax = 691;
    double rightMax = -809;

    // visionmaxxing init
    double targetVel = 0;
    double targetHood = 1;
    double yaw, distance;

    public ILT() {
        addComponents(
                BindingsComponent.INSTANCE,
                new SubsystemComponent(
                        multiFunctionSubsystem.INSTANCE,
                        outtakeSubsystem.INSTANCE,
                        intakeSubsystem.INSTANCE,
                        transferSubsystem.INSTANCE,
                        hoodSubsystem.INSTANCE,
                        turretSubsystem.INSTANCE // Used for kinematic turret logic
                ),
                BulkReadComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }

    // velo/hood calcs
    public double getDistance(double ta) { return 75.16142 * Math.pow(ta, -0.47932); }
    public double getHoodAngle(double dist) { return 0.00578881 * dist - 0.00802588; }
    public double getVelocity(double dist) { return 10.81866 * dist + 1084.95409; }

    // turret calcs

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
        clanka.setStartingPose(new Pose(72, 72, Math.toRadians(90)));

        transferSubsystem.INSTANCE.toNeutral.schedule();

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
    }

    @Override
    public void onStartButtonPressed() {
        limelight.start();

       //drive
        new MecanumDriverControlled(
                frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor,
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX()
        ).schedule();
        //button maxxing
        button(() -> gamepad1.x).whenBecomesTrue(new SequentialGroup(
                outtakeSubsystem.INSTANCE.closeMid(),
                multiFunctionSubsystem.INSTANCE.transferSequenceM3CloseMid(),
                outtakeSubsystem.INSTANCE.off()
        ));



        button(() -> gamepad1.dpad_left).whenBecomesTrue(multiFunctionSubsystem.INSTANCE.transpherSequencNiga());

    }

    @Override
    public void onUpdate() {
        clanka.update();
        BindingManager.update();

        // robot pose(X, Y, R)
        heading = clanka.getHeading();
        clankerX = clanka.getPose().getX();
        clankerY = clanka.getPose().getY();

        // ll digga
        LLResult llresult = limelight.getLatestResult();
        boolean tagFound = (llresult != null && llresult.isValid());

        if (tagFound) {
            yaw = llresult.getTx();
            distance = getDistance(llresult.getTa());
        } else {
            // field-based distance calculation
            distance = Math.sqrt(Math.pow((blueX - clankerX), 2) + Math.pow((blueY - clankerY), 2)) + offset;
        }

        // turret maxxing
        theta = H2(calcDigger());
        ticks = tickAdjustment(calcDigger());
        controller.setGoal(new KineticState(-ticks));

        KineticState currentState = new KineticState(turret.getCurrentPosition(), turret.getVelocity());
        double turretPower = controller.calculate(currentState);
        turret.setPower(turretPower);

        // subsystem automation velo+hood
        if(gamepad1.right_trigger > 0.1){
            outtakeSubsystem.INSTANCE.setVel(getVelocity(distance)).schedule();
        }
        else{
            outtakeSubsystem.INSTANCE.off().schedule();
        }
        hoodSubsystem.INSTANCE.goon(getHoodAngle(distance)).schedule();

        // intake digger
        if (gamepad1.left_bumper){
            intakeSubsystem.INSTANCE.spit.schedule();
        }
        else if (gamepad1.right_bumper){
            intakeSubsystem.INSTANCE.eat.schedule();
        }
        else{
            intakeSubsystem.INSTANCE.sleep.schedule();
        }


        // telemetry
        telemetry.addData("Distance", distance);
        telemetry.addData("Tag Found", tagFound);
        telemetry.addData("Turret Pos", turret.getCurrentPosition());
        telemetry.addData("Target Ticks", -ticks);
        telemetry.addData("Optimal Velo", getVelocity(distance));
        telemetry.update();
    }
}