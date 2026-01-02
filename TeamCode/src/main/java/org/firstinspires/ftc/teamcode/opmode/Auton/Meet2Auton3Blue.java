package org.firstinspires.ftc.teamcode.opmode.Auton;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.Subsystems.hoodSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.intakeSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.multiFunctionSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.outtakeSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.transferSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.units.Angle;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.TurnTo;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;


@Autonomous(name = "blueFar")

public class Meet2Auton3Blue extends NextFTCOpMode {


    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    private int pathState; // Current autonomous path state (state machine)
    private Timer pathTimer, actionTimer, opmodeTimer;

    private PathChain Path1, Path2, Path3, Path4;


    public Meet2Auton3Blue() {
        addComponents(
                new SubsystemComponent(
                        outtakeSubsystem.INSTANCE,
                        hoodSubsystem.INSTANCE,
                        transferSubsystem.INSTANCE,
                        multiFunctionSubsystem.INSTANCE),
                BulkReadComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }


    private Command autonomousRoutine(){
        return new SequentialGroup(
                new FollowPath(Path1),
                multiFunctionSubsystem.INSTANCE.outtakeSequenceFar(),
                new FollowPath(Path2),
                new FollowPath(Path3),
                new FollowPath(Path4)
        );
    }

    private Command shoot() {
        return new SequentialGroup(
                hoodSubsystem.INSTANCE.close,
                outtakeSubsystem.INSTANCE.close(),
                new Delay(0.1),
                multiFunctionSubsystem.INSTANCE.transferSequence()
        );
    }

    @Override
    public void onInit() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        PedroComponent.follower().setStartingPose(new Pose(60, 20, Math.toRadians(90)));

//        paths = new Meet2Auton3Blue.Paths(PedroComponent.follower()); // Build paths
        outtakeSubsystem.INSTANCE.off().schedule();
        outtakeSubsystem.INSTANCE.noPower().schedule();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Path1 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(60, 21.000),
                                new Pose(57.938, 34.578),
                                new Pose(72.618, 39.460)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(115))
                .build();

        Path2 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(72.618, 39.460),
                                new Pose(60, 35)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(180))
                .build();

        Path3 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(60, 35), new Pose(25, 35))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Path4 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(25, 35), new Pose(66.347, 84.510))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(143))
                .build();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }
    @Override
    public void onStartButtonPressed() {
        opmodeTimer.resetTimer();
        autonomousRoutine().schedule();
    }

    @Override
    public void onUpdate() {
        PedroComponent.follower().update(); // Update Pedro Pathing

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", PedroComponent.follower().getPose().getX());
        panelsTelemetry.debug("Y", PedroComponent.follower().getPose().getY());
        panelsTelemetry.debug("Heading", PedroComponent.follower().getPose().getHeading());
        panelsTelemetry.debug("Time Elapsed", opmodeTimer.getElapsedTimeSeconds());
        panelsTelemetry.debug("Current Velocity", outtakeSubsystem.INSTANCE.getJawn());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {

        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(27.000, 130.000), new Pose(27.000, 130.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(143))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(27.000, 130.000), new Pose(47.073, 35))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(180))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(47.073, 35), new Pose(14.270, 35))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(14.270, 35), new Pose(66.347, 84.510))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(143))
                    .build();
        }
    }

}