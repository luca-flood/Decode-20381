package org.firstinspires.ftc.teamcode.opmode.Subsystems;

import androidx.annotation.NonNull;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;



@Configurable
public class outtakeSubsystem implements Subsystem {

    public static final outtakeSubsystem INSTANCE = new outtakeSubsystem();
    private TelemetryManager panelsTelemetry;

    private boolean offJawn = true;
    private outtakeSubsystem(){ }

    //public static PIDCoefficients PIDcoefficients = new PIDCoefficients(0.05, 0, 0.01);
   
    ControlSystem controlSystem = ControlSystem.builder()
            .velPid(0.007, 0, 0.003)
            .basicFF(0.0004, 0, 0.11)
            .build();
    private MotorEx outtake = new MotorEx("outtake");

    public Command noPower() {
        offJawn = true;
        return new SetPower(outtake, 0);
    };

    public Command off() {
        offJawn = true;
        return new RunToVelocity(controlSystem, 0).requires(this);
    }
    public Command close() {
        offJawn = false;
        return new RunToVelocity(controlSystem, 1500).requires(this);
    }
    public Command closeMid() {
        offJawn = false;
        return new RunToVelocity(controlSystem, 1900).requires(this);
    }
    public Command mid() {
        offJawn = false;
        return new RunToVelocity(controlSystem, 1900).requires(this);
    }
    public Command far() {
        offJawn = false;
        return new RunToVelocity(controlSystem, 2300).requires(this);
    }
    public Command newClose = new RunToVelocity(controlSystem, 1400).requires(this);

    public Command crypticJew = new RunToVelocity(controlSystem, 2100).requires(this);
    public Command auto() {
        offJawn = false;
        return new RunToVelocity(controlSystem, 1500).requires(this);
    }

    @Override
    public void initialize() {
        outtake.setPower(0);
    }

    public double getJawn() {
        return outtake.getVelocity();
    }

    @Override
    public void periodic(){
//        if (offJawn) {
//            outtake.setPower(0);
//        }
//        else {
            outtake.setPower(
                    controlSystem.calculate(outtake.getState()));
//        }
    }
}
