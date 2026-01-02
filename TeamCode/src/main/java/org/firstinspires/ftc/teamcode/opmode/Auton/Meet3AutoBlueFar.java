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
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;


@Autonomous(name = "Meet3BlueFar")

public class Meet3AutoBlueFar extends NextFTCOpMode {


    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    private int pathState; // Current autonomous path state (state machine)
    private Timer pathTimer, actionTimer, opmodeTimer;

    private PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9;

    Limelight3A limelight;
    double yaw;

    double distance;

    double delay;

    Follower follower;


    public Meet3AutoBlueFar() {
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
                new FollowPath(Path1)
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
        follower = PedroComponent.follower();
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower.setStartingPose(new Pose(56, 8, Math.toRadians(90)));

//        paths = new Meet2Auton3Blue.Paths(PedroComponent.follower()); // Build paths
        outtakeSubsystem.INSTANCE.off().schedule();
        outtakeSubsystem.INSTANCE.noPower().schedule();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(56.000, 8.000), new Pose(56.000, 36.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))
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
    }

}