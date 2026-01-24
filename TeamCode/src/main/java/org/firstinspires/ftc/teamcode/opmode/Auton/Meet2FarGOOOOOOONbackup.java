package org.firstinspires.ftc.teamcode.opmode.Auton;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.opmode.Subsystems.hoodSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.multiFunctionSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.outtakeSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.transferSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.BLMSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.FLMSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.BRMSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.FRMSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelDeadlineGroup;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
@Disabled

@Autonomous(name = "Meet2FarGOOOOOOONbackup")
public class Meet2FarGOOOOOOONbackup extends NextFTCOpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance

    private PathChain Path1;

    public Meet2FarGOOOOOOONbackup() {
        addComponents(
                new SubsystemComponent(
                        outtakeSubsystem.INSTANCE,
                        hoodSubsystem.INSTANCE,
                        transferSubsystem.INSTANCE,
                        multiFunctionSubsystem.INSTANCE,
                        BLMSubsystem.INSTANCE,
                        FLMSubsystem.INSTANCE,
                        BRMSubsystem.INSTANCE,
                        FRMSubsystem.INSTANCE
                        ),
                BulkReadComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }

    public Command bust() {
        return new SequentialGroup(
                multiFunctionSubsystem.INSTANCE.outtakeSequenceFar(),
                new ParallelGroup(
                        BLMSubsystem.INSTANCE.gooo,
                        FLMSubsystem.INSTANCE.gooo,
                        BRMSubsystem.INSTANCE.gooo,
                        FRMSubsystem.INSTANCE.gooo
                )
        );
    }

    public void onInit() {
        PedroComponent.follower().setStartingPose(new Pose(0, 0, Math.toRadians(0)));

        Path1 = PedroComponent.follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(0, 0), new Pose(0, 6.7))
                )
                .setConstantHeadingInterpolation(0)
                .build();
    }

    public void onStartButtonPressed() {
        bust().schedule();
    }

    public void onUpdate() {
        PedroComponent.follower().update();

//        panelsTelemetry.debug("Current Velocity", outtakeSubsystem.INSTANCE.getJawn());
//        panelsTelemetry.update(telemetry);
    }

    public void onStop() {
        transferSubsystem.INSTANCE.toNeutral.schedule();
    }
}
