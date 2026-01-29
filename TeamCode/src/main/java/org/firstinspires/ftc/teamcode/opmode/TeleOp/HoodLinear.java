package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.opmode.Subsystems.hoodSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.intakeSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.multiFunctionSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.outtakeSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.transferSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.turretSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;


@Disabled

@TeleOp(name="Hood Test", group="diddy bals")
public class HoodLinear extends NextFTCOpMode {
    double position = 0;
    double velocity = 0;
    public HoodLinear() {
        addComponents(
                BindingsComponent.INSTANCE,
                new SubsystemComponent(
                        multiFunctionSubsystem.INSTANCE,
                        outtakeSubsystem.INSTANCE,
                        intakeSubsystem.INSTANCE,
                        transferSubsystem.INSTANCE,
                        hoodSubsystem.INSTANCE,
                        turretSubsystem.INSTANCE // used for kinematic turret logic
                ),
                BulkReadComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }
    public void onUpdate(){
        Servo hood = hardwareMap.servo.get("hood");



            if (gamepad1.left_bumper) {
                position += 0.01;
            }
            else if (gamepad1.right_bumper) {
                position -= 0.01;
            }
            hood.setPosition(position);

            if(gamepad1.a){
                velocity -= 10;
            }
            else if(gamepad1.b){
                velocity += 10;
            }
            outtakeSubsystem.INSTANCE.setVel(velocity).schedule();

            if(gamepad1.x){
                transferSubsystem.INSTANCE.toOuttake.schedule();
            }
            else{
                transferSubsystem.INSTANCE.toNeutral.schedule();

            }

            if(gamepad1.right_trigger > 0.1){
                intakeSubsystem.INSTANCE.eat.schedule();
            }
            else{
                intakeSubsystem.INSTANCE.sleep.schedule();

            }

            telemetry.addData("Hood Position - Diddy", hood.getPosition());
            telemetry.addData("velocity",         outtakeSubsystem.INSTANCE.getJawn());

        telemetry.update();

        }
    }

