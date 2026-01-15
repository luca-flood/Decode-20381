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
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.Subsystems.hoodSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.intakeSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.multiFunctionSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.outtakeSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.transferSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.turretSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;


@Autonomous(name = "Meet3BlueNear")

public class Meet3AutoBlueNear extends NextFTCOpMode {


    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    private int pathState; // Current autonomous path state (state machine)
    private Timer pathTimer, actionTimer, opmodeTimer;

    private PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10, Path11;
    double yaw;

    double distance;

    double delay;

    Follower follower;


    public Meet3AutoBlueNear() {
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
                new FollowPath(Path1),
                outtakeSubsystem.INSTANCE.closeMid(),
                multiFunctionSubsystem.INSTANCE.transferSequenceM3CloseMid(),
                intakeSubsystem.INSTANCE.eat,
                new FollowPath(Path2),
                intakeSubsystem.INSTANCE.sleep,
                new Delay(0.5),
                new FollowPath(Path4),
                new Delay(0.5),
                outtakeSubsystem.INSTANCE.closeMid(),
                multiFunctionSubsystem.INSTANCE.transferSequenceM3CloseMid(),
                intakeSubsystem.INSTANCE.eat,
                new FollowPath(Path5),
                intakeSubsystem.INSTANCE.sleep,
                new FollowPath(Path6),
                outtakeSubsystem.INSTANCE.closeMid(),
                multiFunctionSubsystem.INSTANCE.transferSequenceM3CloseMid(),
                new Delay(0.5),
                intakeSubsystem.INSTANCE.eat,
                new FollowPath(Path7),
                intakeSubsystem.INSTANCE.sleep,
                new FollowPath(Path8),
                new Delay(0.5),
                outtakeSubsystem.INSTANCE.closeMid(),
                multiFunctionSubsystem.INSTANCE.transferSequenceM3CloseMid()

        );
    }


    @Override
    public void onInit() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = PedroComponent.follower();
        follower.setStartingPose(new Pose(24, 130, Math.toRadians(143)));

        outtakeSubsystem.INSTANCE.off().schedule();
        outtakeSubsystem.INSTANCE.noPower().schedule();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(24.939, 128.313), new Pose(48.872, 103.173))
                )
                .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(143))
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(48.872, 103.173),
                                new Pose(60.335, 81.855),
                                new Pose(18.101, 89.296)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(180))
                .build();

        Path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(18.101, 89.296), new Pose(48.268, 103.173))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(143))
                .build();

        Path5 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(48.268, 103.173),
                                new Pose(68.983, 57.520),
                                new Pose(16.894, 60.335)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(180))
                .build();

        Path6 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(16.894, 60.335), new Pose(48.268, 102.972))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(143))
                .build();

        Path7 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(48.268, 102.972),
                                new Pose(91.307, 28.156),
                                new Pose(15.486, 34.793)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(180))
                .build();

        Path8 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(15.486, 34.793), new Pose(48.469, 103.374))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(143))
                .build();
    }
    @Override
    public void onStartButtonPressed() {
        opmodeTimer.resetTimer();

        autonomousRoutine().schedule();
    }

    @Override
    public void onUpdate() {
        PedroComponent.follower().update(); // Update Pedro Pathing

        BindingManager.update();

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", PedroComponent.follower().getPose().getX());
        panelsTelemetry.debug("Y", PedroComponent.follower().getPose().getY());
        panelsTelemetry.debug("Heading", PedroComponent.follower().getPose().getHeading());
        panelsTelemetry.debug("Time Elapsed", opmodeTimer.getElapsedTimeSeconds());
        panelsTelemetry.debug("Current Velocity", outtakeSubsystem.INSTANCE.getJawn());
        panelsTelemetry.update(telemetry);
    }

}