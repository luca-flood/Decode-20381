package org.firstinspires.ftc.teamcode.opmode.Auton;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
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

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;


@Autonomous(name = "Red Classifier Goon", group = "dit shigger")
public class ILTclassifierBlueRed extends NextFTCOpMode {

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



    public ILTclassifierBlueRed() {
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

    private Command autonomousRoutine(){
        return new SequentialGroup(
                intakeSubsystem.INSTANCE.eat,
                outtakeSubsystem.INSTANCE.setVel(1580),
                new ParallelGroup(
                        new SequentialGroup(
                                multiFunctionSubsystem.INSTANCE.transpherSequencNiga(),
                                intakeSubsystem.INSTANCE.eat,
                                hoodSubsystem.INSTANCE.goon(.15),
                                outtakeSubsystem.INSTANCE.setVel(2120),
                                intakeSubsystem.INSTANCE.eat,
                                multiFunctionSubsystem.INSTANCE.transpherSequencNiga(),
                                hoodSubsystem.INSTANCE.goon(.15),
                                new Delay(0.1),
                                intakeSubsystem.INSTANCE.eat,
                                multiFunctionSubsystem.INSTANCE.transpherSequencNiga()
                        ),
                        new FollowPath(Path1, true, 0.8)
                ),
               // outtakeSubsystem.INSTANCE.off(),
                intakeSubsystem.INSTANCE.slowSuck,
                new ParallelGroup(
                        (Command) new FollowPath(Path2),
                        outtakeSubsystem.INSTANCE.setVel(1900),
                        hoodSubsystem.INSTANCE.goon(.20)
                        //ILTclassifierBlue.decreaseX()
                ),
                intakeSubsystem.INSTANCE.eat,
                multiFunctionSubsystem.INSTANCE.transpherSequencNiga(),
                hoodSubsystem.INSTANCE.goon(.33),
                new Delay (0.2),
                multiFunctionSubsystem.INSTANCE.transpherSequencNiga(),
                hoodSubsystem.INSTANCE.goon(31),
                new Delay (0.1),
                multiFunctionSubsystem.INSTANCE.transpherSequencNiga(),
                hoodSubsystem.INSTANCE.goon(.29),
                outtakeSubsystem.INSTANCE.setVel(1700),
                intakeSubsystem.INSTANCE.slowSuck,
                new FollowPath(Path3),
                new Delay(1),

                new ParallelGroup(
                        new FollowPath(Path4),
                        outtakeSubsystem.INSTANCE.setVel(1900),
                        hoodSubsystem.INSTANCE.goon(.20)
                ),
                intakeSubsystem.INSTANCE.eat,
                multiFunctionSubsystem.INSTANCE.transpherSequencNiga(),
                hoodSubsystem.INSTANCE.goon(.36),
                new Delay (0.2),
                multiFunctionSubsystem.INSTANCE.transpherSequencNiga(),
                hoodSubsystem.INSTANCE.goon(31),
                new Delay (0.1),
                multiFunctionSubsystem.INSTANCE.transpherSequencNiga(),
                hoodSubsystem.INSTANCE.goon(.29),
                outtakeSubsystem.INSTANCE.setVel(1700),
                intakeSubsystem.INSTANCE.slowSuck,
                new FollowPath(Path3),
                new Delay(1),

                new ParallelGroup(
                        new FollowPath(Path4),
                        outtakeSubsystem.INSTANCE.setVel(1900),
                        hoodSubsystem.INSTANCE.goon(.20)
                ),
                intakeSubsystem.INSTANCE.eat,
                multiFunctionSubsystem.INSTANCE.transpherSequencNiga(),
                hoodSubsystem.INSTANCE.goon(.36),
                new Delay (0.2),
                multiFunctionSubsystem.INSTANCE.transpherSequencNiga(),
                hoodSubsystem.INSTANCE.goon(31),
                new Delay (0.1),
                multiFunctionSubsystem.INSTANCE.transpherSequencNiga(),
                hoodSubsystem.INSTANCE.goon(.29),
                outtakeSubsystem.INSTANCE.setVel(1700),
                intakeSubsystem.INSTANCE.slowSuck,
                new FollowPath(Path5),
                new Delay(1),
                new ParallelGroup(
                    new FollowPath(Path6),
                    outtakeSubsystem.INSTANCE.setVel(1900),
                    hoodSubsystem.INSTANCE.goon(.20)
                ),
                intakeSubsystem.INSTANCE.eat,
                multiFunctionSubsystem.INSTANCE.transpherSequencNiga(),
                hoodSubsystem.INSTANCE.goon(.36),
                new Delay (0.2),
                multiFunctionSubsystem.INSTANCE.transpherSequencNiga(),
                hoodSubsystem.INSTANCE.goon(31),
                new Delay (0.1),
                multiFunctionSubsystem.INSTANCE.transpherSequencNiga(),
                hoodSubsystem.INSTANCE.goon(.29),
                outtakeSubsystem.INSTANCE.setVel(0),
                intakeSubsystem.INSTANCE.sleep,
                new FollowPath(Path11)


//                intakeSubsystem.INSTANCE.sleep
//                new FollowPath(Path2),
//                new FollowPath(Path3)

        );
    }


    @Override
    public void onInit() {
        hoodSubsystem.INSTANCE.goon(.75).schedule();
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
        clanka.setStartingPose(new Pose(24, 130, Math.toRadians(143)));

        outtakeSubsystem.INSTANCE.off().schedule();
        outtakeSubsystem.INSTANCE.noPower().schedule();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        Path1 = clanka
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(24.000, 130.000),
                                new Pose(105.600, 55.400),
                                new Pose(20.800, 63.800)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(180))
                .build();

        Path2 = clanka
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(20.800, 63.800), new Pose(58.800, 79.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(143))
                .build();

        Path3 = clanka
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(58.800, 79.000),
                                new Pose(15.000, 60.000),
                                new Pose(9.500, 62.670)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(140))
                .build();
        Path4 = clanka
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(9.500, 62.670),
                                new Pose(55.800, 79.500)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(143))
                .build();

        Path5 = clanka
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(55.800, 79.500),
                                new Pose(9.500, 62.900)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(140))
                .build();

        Path6 = clanka
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(9.500, 62.900),
                                new Pose(55.800,79.500)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(143))
                .build();

        Path11 = clanka
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(55.800,79.500),
                                new Pose(40, 79.500)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(143))
                .build();
    }

    @Override
    public void onStartButtonPressed() {
        opmodeTimer.resetTimer();

        autonomousRoutine().schedule();
    }

    @Override
    public void onUpdate() {
        clanka.update();

        theta = H2(calcDigger());
        ticks = tickAdjustment(calcDigger());
        controller.setGoal(new KineticState(-ticks));

        KineticState currentState = new KineticState(turret.getCurrentPosition(), turret.getVelocity());
        double turretPower = controller.calculate(currentState);
        turret.setPower(turretPower);

        clankerX = clanka.getPose().getX();
        clankerY = clanka.getPose().getY();
        clankerR = clanka.getHeading();

        distance = Math.sqrt(Math.pow((blueX - clankerX), 2) + Math.pow((blueY - clankerY), 2)) + offset;

        BindingManager.update();

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("X", clanka.getPose().getX());
        panelsTelemetry.debug("Y", clanka.getPose().getY());
        panelsTelemetry.debug("Heading", clanka.getPose().getHeading() * 180 / Math.PI);
        panelsTelemetry.debug("Optimal Hood Angle", H2(calcDigger()));
        panelsTelemetry.debug("Ticks", tickAdjustment(calcDigger()));
//        panelsTelemetry.debug("Velocityyuh", outtakeSubsystem.INSTANCE.getJawn());
        panelsTelemetry.debug("X", goalX);
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
        return 0.00578881 * distance - 0.00802588;
    }

    public double getVelocity(double distance) {
        return 10.81866 * distance + 1084.95409;
    }

    public double getVel(double dist) {
        return 7.65268 * dist + 1123.06996;
    }
    public double getHood(double dist) {
        return 71.71201 * Math.pow(dist, -1.17652);
    }

    public Command increaseX(){goalX += 0.5; return null; };
    Command decreaseX = new LambdaCommand()
            .setStart(() -> {
                goalX -= 0.5;
    })
            .setIsDone(() -> true)
            .requires(this)
            .named("decreaseX");


}
