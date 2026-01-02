package org.firstinspires.ftc.teamcode.opmode.Subsystems;


import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.builder.ControlSystemBuilder;
import dev.nextftc.control.feedback.AngleType;
import dev.nextftc.control.interpolators.FirstOrderEMAParameters;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;

import dev.nextftc.hardware.impl.MotorEx;

import dev.nextftc.hardware.powerable.SetPower;

public class turretSubsystemLimelight implements Subsystem {

    private Limelight3A limelight;

    public static final turretSubsystemLimelight INSTANCE = new turretSubsystemLimelight();
    private TelemetryManager panelsTelemetry;
    public double currentYaw = 0.0;
    private turretSubsystemLimelight(){ }

    FirstOrderEMAParameters emaParams = new FirstOrderEMAParameters(0.2, new KineticState());


    public ControlSystem controlSystem = ControlSystem.builder()
            .angular(AngleType.DEGREES,
                    feedback ->
                            feedback.posPid(0.08, 0.0, 0.04)
            )
          .posFilter(
                   filterBuilder ->
                            filterBuilder.lowPass(0.2).lowPass(0.4))
            .basicFF(0.01, 0.0, 0.2)
            .emaInterpolator(emaParams)
            .build();

    private MotorEx turret= new MotorEx("turret");

    public void updateAngle(double yaw) {
        this.currentYaw = yaw;
    }
    public void updateAngleCoast(double yaw) {
        this.currentYaw = yaw*0.4;
    }

    @Override
    public void initialize() {
        turret.setPower(0);
        controlSystem.setGoal(new KineticState(0.0, 0.0, 0.0));
    }

    public Command setPower(double pow) {
        return new SetPower(turret, pow);
    }

    @Override
    public void periodic(){
        KineticState currentState = new KineticState(currentYaw, 0, 0.0);
        double power = controlSystem.calculate(currentState);
        turret.setPower(-power);
    }
}