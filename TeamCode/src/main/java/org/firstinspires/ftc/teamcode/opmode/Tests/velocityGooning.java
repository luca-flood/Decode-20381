package org.firstinspires.ftc.teamcode.opmode.Tests;

import static dev.nextftc.bindings.Bindings.button;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmode.Subsystems.outtakeSubsystem;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;


@Disabled

@TeleOp(name="veloGoon")
public class velocityGooning extends NextFTCOpMode {

    public velocityGooning() {
        addComponents(
                BindingsComponent.INSTANCE,
                new SubsystemComponent(
                        outtakeSubsystem.INSTANCE
                ),
                BulkReadComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {
    }

    public void onStartButtonPressed() {
        Button gamepad1a = button(() -> gamepad1.a);
        gamepad1a
                .whenBecomesTrue(
                        outtakeSubsystem.INSTANCE.crypticJew
                );

        Button gamepad1b = button(() -> gamepad1.b);
        gamepad1b
                .whenBecomesTrue(
                        outtakeSubsystem.INSTANCE.off()
                );
    }

    public void onUpdate() {
        BindingManager.update();

        telemetry.addData("Current Velocity 1",outtakeSubsystem.INSTANCE.getJawn());
        telemetry.update();
    }

}
