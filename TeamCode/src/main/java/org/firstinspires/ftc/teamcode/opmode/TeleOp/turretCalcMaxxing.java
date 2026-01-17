package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.opmode.Subsystems.turretSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.DriverControlledCommand;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp(name="turretKinematics", group="Linear Opmode")
public class turretCalcMaxxing extends NextFTCOpMode {

    MotorEx turret = new MotorEx("turret").zeroed().reversed();
    private ControlSystem controller;
    double clankerX;
    double clankerY;

    double goalX = 20;
    double goalY = 115;
    double heading = 90;
    double theta;
    double ticks;

    private final MotorEx frontLeftMotor = new MotorEx("leftFront").reversed();
    private final MotorEx frontRightMotor = new MotorEx("rightFront");
    private final MotorEx backLeftMotor = new MotorEx("leftRear").reversed();
    private final MotorEx backRightMotor = new MotorEx("rightRear");

    public turretCalcMaxxing(){
        addComponents(
                BindingsComponent.INSTANCE,
                new SubsystemComponent(
                        turretSubsystem.INSTANCE
//    //                    transferSubsystem.INSTANCE,
//    //                    hoodSubsystem.INSTANCE,
//    //                    intakeSubsystem.INSTANCE,
//    //                    outtakeSubsystem.INSTANCE,
//    //                    multiFunctionSubsystem.INSTANCE
                        ),
                BulkReadComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }

    @Override
    public void onInit(){
        PedroComponent.follower().setStartingPose(new Pose(72, 72, Math.toRadians(90)));
        turret.setCurrentPosition(0);
        turret.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        controller = ControlSystem.builder()
                .posPid(0.03, 0.0, 0.0)
                .basicFF(0, 0, 0.1)
                .build();

        frontLeftMotor.setDirection(1);
        backLeftMotor.setDirection(1);
        frontRightMotor.setDirection(1);
        backRightMotor.setDirection(1);

        controller.setGoal(new KineticState(0.0));
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
        driverControlled.schedule();
    }

    public void onUpdate() {
        PedroComponent.follower().update();
        theta = H2(calcDigger());
        ticks = tickAdjustment(calcDigger());
        heading = PedroComponent.follower().getHeading();
        clankerX = PedroComponent.follower().getPose().getX();
        clankerY = PedroComponent.follower().getPose().getY();
        telemetry.addData("Current Bot X", clankerX);
        telemetry.addData("Current Boy Y", clankerY);
        telemetry.addData("Heading", heading * 180 / Math.PI);
        telemetry.addData("X distance", goalX - clankerX);
        telemetry.addData("Y distance", goalY - (4 + clankerY));
        telemetry.addData("Digger", calcDigger());
        telemetry.addData("Current Position", turret.getCurrentPosition());
        telemetry.addData("Needed Angle Adjustment", theta);
        telemetry.addData("Tick Adjustment", tickAdjustment(calcDigger()));
        telemetry.update();
        if (gamepad1.a) {
            controller.setGoal(new KineticState(-ticks));
        }
        else if (gamepad1.b) {
            controller.setGoal(new KineticState(0));
        }

        KineticState currentState = new KineticState(
                turret.getCurrentPosition(), turret.getVelocity()
        );
        double power = controller.calculate(currentState);
        turret.setPower(power);
    }

    public double atan2(double y, double x) {
        if (x > 0) {
            return Math.atan(y / x);
        }
        else if (x < 0) {
            if (y >= 0) {
                return Math.atan(y / x) + Math.PI;
            }
            else {
                return Math.atan(y / x) - Math.PI;
            }
        }
        else {
            if (y > 0) {
                return 0.5 * Math.PI;
            }
            else if (y < 0) {
                return -0.5 * Math.PI;
            }
            else {
                return Math.tan(0.5 * Math.PI);
            }
        }
    }

    public double calcDigger() {
        return atan2(goalY - (2 + clankerY), goalX - clankerX);
    }
    public double H1(double digger) {
        return digger * 180 / Math.PI;
    }

    public double H2(double digger) {
        return H1(digger) - heading * 180 / Math.PI;
    }

    public double tickAdjustment(double digger) {
        return H2(digger) * 130/36*384.5/360;
    }

}


