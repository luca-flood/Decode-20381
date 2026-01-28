package org.firstinspires.ftc.teamcode.opmode.Auton;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.opmode.Subsystems.hoodSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.intakeSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.multiFunctionSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.outtakeSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.transferSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.turretSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.TurnTo;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;

@Autonomous(name = "ILT Blue 2 Classifier Far")
public class ILTFar2BlueClassifier extends NextFTCOpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance

    private Timer pathTimer, actionTimer, opmodeTimer;

    private PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10, Path11;


    MotorEx turret = new MotorEx("turret").zeroed().reversed();
    private ControlSystem controller;
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
    double goalX = 13.8;
    double goalY = 144;
    double theta;
    double ticks;
    Follower clanka;

    Limelight3A limelight;
    double leftMax = 691;
    double rightMax = -809;

    double robotVelocityMag = 0;
    double robotVelocityXComp = 0;
    double robotVelocityYComp = 0;

    boolean limelightTracking = false;
    boolean hasCorrectedLL = false;
    double finalTargetTicks;
    boolean far = false;
    boolean readyShoot = false;
    double ticksToDegrees = (130.0 / 36.0) * (384.5 / 360.0);

    double shootingHeading = 110;

    public ILTFar2BlueClassifier() {
        addComponents(
                BindingsComponent.INSTANCE,
                new SubsystemComponent(
                        turretSubsystem.INSTANCE,
                        transferSubsystem.INSTANCE,
                        hoodSubsystem.INSTANCE,
                        intakeSubsystem.INSTANCE,
                        outtakeSubsystem.INSTANCE,
                        multiFunctionSubsystem.INSTANCE),
                BulkReadComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }

    private Command autonomousRoutine() {
        return new SequentialGroup(
                // Shoot 3
                outtakeSubsystem.INSTANCE.setVel(2440),
                hoodSubsystem.INSTANCE.goon(.15),
                intakeSubsystem.INSTANCE.slowSuck,
                multiFunctionSubsystem.INSTANCE.transpherSequencNiga(),
                hoodSubsystem.INSTANCE.goon(.2),
                new Delay(.5),
                multiFunctionSubsystem.INSTANCE.transpherSequencNiga(),
                new Delay(0.5),
                multiFunctionSubsystem.INSTANCE.transpherSequencNiga(),

                // Grab Middle Row
                outtakeSubsystem.INSTANCE.setVel(0),
                intakeSubsystem.INSTANCE.eat,
                new FollowPath(Path3),
                intakeSubsystem.INSTANCE.sleep,

                // Go Back to Shooting Spot
                new FollowPath(Path4, true, 0.8),

                // Shoot 3
                outtakeSubsystem.INSTANCE.setVel(2440),
                hoodSubsystem.INSTANCE.goon(.15),
                intakeSubsystem.INSTANCE.slowSuck,
                multiFunctionSubsystem.INSTANCE.transpherSequencNiga(),
                hoodSubsystem.INSTANCE.goon(.2),
                new Delay(.5),
                multiFunctionSubsystem.INSTANCE.transpherSequencNiga(),
                hoodSubsystem.INSTANCE.goon(.25),
                new Delay(0.4),
                multiFunctionSubsystem.INSTANCE.transpherSequencNiga(),
                intakeSubsystem.INSTANCE.sleep,

                // Grab Classifier Balls
                outtakeSubsystem.INSTANCE.setVel(0),
                new FollowPath(Path6),
                intakeSubsystem.INSTANCE.eat,
                new Delay(2),
                intakeSubsystem.INSTANCE.sleep,

                // Go Back to Shooting Spot
                new FollowPath(Path7),

                // Shoot 3
                intakeSubsystem.INSTANCE.slowSuck,
                outtakeSubsystem.INSTANCE.setVel(2440),
                hoodSubsystem.INSTANCE.goon(.15),
                intakeSubsystem.INSTANCE.slowSuck,
                multiFunctionSubsystem.INSTANCE.transpherSequencNiga(),
                hoodSubsystem.INSTANCE.goon(.2),
                new Delay(.5),
                multiFunctionSubsystem.INSTANCE.transpherSequencNiga(),
                hoodSubsystem.INSTANCE.goon(.25),
                new Delay(0.4),
                multiFunctionSubsystem.INSTANCE.transpherSequencNiga(),

                // Grab Top Row
                intakeSubsystem.INSTANCE.sleep,
                outtakeSubsystem.INSTANCE.setVel(0),
                intakeSubsystem.INSTANCE.eat,
                new FollowPath(Path1),
                intakeSubsystem.INSTANCE.sleep,

                // Go Back to Shoot Spot
                new FollowPath(Path2, true, 0.8),

                // Shoot 3
                outtakeSubsystem.INSTANCE.setVel(2440),
                hoodSubsystem.INSTANCE.goon(.15),
                intakeSubsystem.INSTANCE.slowSuck,
                multiFunctionSubsystem.INSTANCE.transpherSequencNiga(),
                hoodSubsystem.INSTANCE.goon(.2),
                new Delay(.5),
                multiFunctionSubsystem.INSTANCE.transpherSequencNiga(),
                hoodSubsystem.INSTANCE.goon(.25),
                new Delay(0.4),
                multiFunctionSubsystem.INSTANCE.transpherSequencNiga(),

                // Leave
                intakeSubsystem.INSTANCE.sleep,
                outtakeSubsystem.INSTANCE.setVel(0),
                new FollowPath(Path5)
        );
    }

    @Override
    public void onInit() {
        transferSubsystem.INSTANCE.toNeutral.schedule();
        hoodSubsystem.INSTANCE.goon(0).schedule();

        opmodeTimer = new Timer();

        clanka = PedroComponent.follower();
        transferSubsystem.INSTANCE.toNeutral.schedule();

        turret.setCurrentPosition(0);
        turret.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        controller = ControlSystem.builder()
                .posPid(0.016, 0.0, 0.0001)
                .basicFF(0, 0, 0.1)
                .build();
        controller.setGoal(new KineticState(0.0));

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        clanka.setStartingPose(new Pose(54, 8, Math.toRadians(90)));

        outtakeSubsystem.INSTANCE.off().schedule();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        telemetry.setMsTransmissionInterval(1);

        Path1 = clanka.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(56.000, 8.000),
                                new Pose(96.285, 42.684),
                                new Pose(20.631, 34.765)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))

                .build();

        Path2 = clanka.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(24.631, 34.765),

                                new Pose(55.707, 12.223)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(shootingHeading))

                .build();

        Path3 = clanka.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(55.707, 12.223),
                                new Pose(77.955, 64.469),
                                new Pose(20.129, 59.460)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))

                .build();

        Path4 = clanka.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(24.129, 59.460),
                        new Pose(77.955, 64.469),
                        new Pose(54.854, 12.468)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(shootingHeading))
                .build();

        Path5 = clanka
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(55.388, 12), new Pose(50.388, 40))
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        Path6 = clanka.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(56.000, 8.000),
                                new Pose(91.784, 75.452),
                                new Pose(11.336, 65.019)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(140))

                .build();

        Path7 = clanka.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(11.336, 61.019),
                                new Pose(91.784, 75.452),
                                new Pose(56.093, 11.436)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(shootingHeading + 3))

                .build();
    }

    @Override
    public void onStartButtonPressed() {
        opmodeTimer.resetTimer();
        limelight.start();
        autonomousRoutine().schedule();
    }

    @Override
    public void onUpdate() {
        clanka.update();

        clankerX = clanka.getPose().getX();
        clankerY = clanka.getPose().getY();
        clankerR = clanka.getHeading();
        distance = Math.sqrt(Math.pow((blueX - clankerX), 2) + Math.pow((blueY - clankerY), 2)) + offset;

        targetVel = getVel(distance);
        targetHood = getVel(distance);

        robotVelocityMag = clanka.getVelocity().getMagnitude();
        robotVelocityXComp = clanka.getVelocity().getXComponent();
        robotVelocityYComp = clanka.getVelocity().getMagnitude();

        readyShoot = Math.abs(robotVelocityMag) < 4;

        // ll digga
        LLResult result = limelight.getLatestResult();
        boolean tagFound = (result != null && result.isValid());


        distance = Math.sqrt(Math.pow((blueX - clankerX), 2) + Math.pow((blueY - clankerY), 2)) + offset;

        if(readyShoot) {
            if (!hasCorrectedLL) {
                if (tagFound) {
                    yaw = result.getTx();
                    double relativeTickOffset = yaw * ticksToDegrees;
                    finalTargetTicks = turret.getCurrentPosition() + relativeTickOffset;
                    hasCorrectedLL = true;
                } else {
                    theta = H2(calcDigger());
                    ticks = tickAdjustment(calcDigger());
                    finalTargetTicks = -ticks;
                }
            }
        }
        else {
            theta = H2(calcDigger());
            ticks = tickAdjustment(calcDigger());
            finalTargetTicks = -ticks;
            hasCorrectedLL = false;
        }

        if (opmodeTimer.getElapsedTimeSeconds() > 5) {
            finalTargetTicks = 0;
        }

        controller.setGoal(new KineticState(finalTargetTicks));

        KineticState currentState = new KineticState(turret.getCurrentPosition(), turret.getVelocity());
        double turretPower = controller.calculate(currentState);
        turret.setPower(turretPower);

        panelsTelemetry.debug("Tag Found", tagFound);
        panelsTelemetry.debug("Has Corrected", hasCorrectedLL);
        panelsTelemetry.debug("Error", yaw);
        panelsTelemetry.debug("Clanker Vel", robotVelocityMag);
        panelsTelemetry.update(telemetry);
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
        return normalize(digger - clankerR) * 180 / Math.PI;
    }

    public double tickAdjustment(double digger) {
        double calcTicks = H2(digger) * (130.0 / 36.0) * (384.5 / 360.0);
        double ticksPerRev = 384.5 * (130.0 / 36.0);

        if (leftMax != 0 && calcTicks > leftMax) calcTicks -= ticksPerRev;
        else if (rightMax != 0 && calcTicks < rightMax) calcTicks += ticksPerRev;

        return calcTicks;
    }

    public double getDistance(double ta) {
        return 75.16142 * Math.pow(ta, -0.47932);
    }
    public double getHoodAngle(double distance) {
        return (distance >= 120) ? -0.0210202 * distance +3.48553 : 0.00578881 * distance - 0.00802588;
    }

    public double getVelocity(double distance) {
        return (distance >= 120) ? 19.17589 * distance - 582.51652 : 10.81866 * distance + 1084.95409;
    }

    public double getVel(double dist) {
        return 7.65268 * dist + 1123.06996;
    }
    public double getHood(double dist) {
        return 71.71201 * Math.pow(dist, -1.17652);
    }

    public Command increaseX(){goalX += 0.5; return null; };
}
