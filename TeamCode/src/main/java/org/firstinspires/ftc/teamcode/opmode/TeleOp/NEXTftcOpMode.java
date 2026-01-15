package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.conditionals.IfElseCommand;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
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
import static dev.nextftc.bindings.Bindings.*;

@Disabled

@TeleOp(name = "NextFTC TeleOp Program Java")
public class NEXTftcOpMode extends NextFTCOpMode {


    public NEXTftcOpMode() {
        addComponents(
                BindingsComponent.INSTANCE,
                new SubsystemComponent (
                        hoodSubsystem.INSTANCE,
                        intakeSubsystem.INSTANCE,
                        outtakeSubsystem.INSTANCE,
                        multiFunctionSubsystem.INSTANCE),


                BulkReadComponent.INSTANCE
        );
    }

    // change the names and directions to suit your robot
    private final MotorEx frontLeftMotor = new MotorEx("leftFront").reversed();
    private final MotorEx frontRightMotor = new MotorEx("rightFront");
    private final MotorEx backLeftMotor = new MotorEx("leftRear").reversed();
    private final MotorEx backRightMotor = new MotorEx("rightRear");


    @Override
    public void onInit(){
        multiFunctionSubsystem.INSTANCE.stopPlease().schedule();
        transferSubsystem.INSTANCE.toNeutral.schedule();

    }

    @Override
    public void onWaitForStart() {

    }

    @Override
    public void onStartButtonPressed() {
        DriverControlledCommand driverControlled = new MecanumDriverControlled(
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX()
        );

//        Gamepads.gamepad1().a()
//                .whenBecomesTrue(
//                        multiFunctionSubsystem.INSTANCE.outtakeSequence()
//                );

        Gamepads.gamepad1().leftTrigger().atLeast(0.1).whenTrue(
                outtakeSubsystem.INSTANCE.off()
        );

        Gamepads.gamepad1().rightTrigger().atLeast(0.1)
                .whenBecomesTrue(outtakeSubsystem.INSTANCE.mid())
                .whenTrue(outtakeSubsystem.INSTANCE.mid())
                .whenBecomesFalse(outtakeSubsystem.INSTANCE.off());

        Gamepads.gamepad2().rightTrigger().atLeast(0.1)
                .whenBecomesTrue(outtakeSubsystem.INSTANCE.mid())
                .whenTrue(outtakeSubsystem.INSTANCE.mid())
                .whenBecomesFalse(outtakeSubsystem.INSTANCE.off());

        Button gamepad1leftBumper = button(() -> gamepad1.left_bumper);
        gamepad1leftBumper
                .whenBecomesTrue(intakeSubsystem.INSTANCE.spit)
                .whenTrue(intakeSubsystem.INSTANCE.spit)
                .whenBecomesFalse(intakeSubsystem.INSTANCE.sleep);

        Button gamepad1rightBumper = button(() -> gamepad1.right_bumper);
        gamepad1rightBumper
                .whenBecomesTrue(intakeSubsystem.INSTANCE.eat)
                .whenTrue(intakeSubsystem.INSTANCE.eat)
                .whenBecomesFalse(intakeSubsystem.INSTANCE.sleep);

        Button gamepad2leftBumper = button(() -> gamepad2.left_bumper);
        gamepad1leftBumper
                .whenBecomesTrue(intakeSubsystem.INSTANCE.spit)
                .whenTrue(intakeSubsystem.INSTANCE.spit)
                .whenBecomesFalse(intakeSubsystem.INSTANCE.sleep);

        Button gamepad2rightBumper = button(() -> gamepad2.right_bumper);
        gamepad1rightBumper
                .whenBecomesTrue(intakeSubsystem.INSTANCE.eat)
                .whenTrue(intakeSubsystem.INSTANCE.eat)
                .whenBecomesFalse(intakeSubsystem.INSTANCE.sleep);




        Button gamepad1y = button(() -> gamepad1.y);
        gamepad1y
                .whenBecomesTrue(multiFunctionSubsystem.INSTANCE.transpherSequencNiga());

        Button gamepad1Dleft = button(() -> gamepad1.dpad_left);
        gamepad1Dleft
                .whenBecomesTrue(multiFunctionSubsystem.INSTANCE.outtakeSequence());

        Button gamepad1a = button(() -> gamepad1.a);
        gamepad1a
                .whenBecomesTrue(hoodSubsystem.INSTANCE.one);

        Button gamepad2y = button(() -> gamepad2.y);
        gamepad1y
                .whenBecomesTrue(multiFunctionSubsystem.INSTANCE.transpherSequencNiga());








        /*
        Gamepads.gamepad1().leftBumper().whenTrue(
                intakeSubsystem.INSTANCE.eat
        );

        Gamepads.gamepad1().leftBumper().whenTrue(
                intakeSubsystem.INSTANCE.spit
        );

         */

//        Gamepads.gamepad1().x().whenBecomesTrue(
//                transferSubsystem.INSTANCE.toOuttake
//        );
//        Gamepads.gamepad1().y().whenBecomesTrue(
//                transferSubsystem.INSTANCE.toNeutral
//        );



        driverControlled.schedule();

    }


    @Override
    public void onUpdate() {

        BindingManager.update();
        telemetry.addData("Current Velocity: ", outtakeSubsystem.INSTANCE.getJawn());
        telemetry.addData("Current Pose: ", transferSubsystem.INSTANCE.getJawn());
        telemetry.update();
        super.onUpdate();// updates command bindings
    }

    @Override
    public void onStop() {
        BindingManager.reset();
        super.onStop();
    }
}