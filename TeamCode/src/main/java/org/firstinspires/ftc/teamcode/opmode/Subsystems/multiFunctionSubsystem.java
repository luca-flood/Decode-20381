package org.firstinspires.ftc.teamcode.opmode.Subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.core.commands.delays.Delay;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

public class multiFunctionSubsystem implements Subsystem {

    public static final multiFunctionSubsystem INSTANCE = new multiFunctionSubsystem();
    private TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    private multiFunctionSubsystem(){ }
    public Command outtakeSequence() {
        return new SequentialGroup(
                hoodSubsystem.INSTANCE.close,
                transferSubsystem.INSTANCE.toNeutral,
                outtakeSubsystem.INSTANCE.newClose,
                new Delay (0.1),
                transferSequence(),
                outtakeSubsystem.INSTANCE.off()
        ).requires(this);
    }

    public Command transferSequenceM3() {
        return new SequentialGroup(
                hoodSubsystem.INSTANCE.closeMid,
                new Delay(0.5),
                intakeSubsystem.INSTANCE.eat,
                new Delay(0.2),
                intakeSubsystem.INSTANCE.sleep,
                transferSubsystem.INSTANCE.toNeutral,
//                outtakeSubsystem.INSTANCE.newClose,
                new Delay(0.1),
                transferSubsystem.INSTANCE.toOuttake,
                new Delay(0.2),
                transferSubsystem.INSTANCE.toNeutral,
                intakeSubsystem.INSTANCE.eat,
                new Delay(0.9),
                transferSubsystem.INSTANCE.toOuttake,
                new Delay(0.2),
                transferSubsystem.INSTANCE.toNeutral,
                hoodSubsystem.INSTANCE.finalShot,
                new Delay(0.7),
                transferSubsystem.INSTANCE.toOuttake,
                new Delay(0.2),
                intakeSubsystem.INSTANCE.sleep,
                outtakeSubsystem.INSTANCE.off(),
                transferSubsystem.INSTANCE.toNeutral
        ).requires(this);
    }

    public Command transferSequenceM3CloseMid() {
        return new SequentialGroup(
                hoodSubsystem.INSTANCE.closeMid,
                intakeSubsystem.INSTANCE.eat,
                new Delay(0.2),
                intakeSubsystem.INSTANCE.sleep,
                transferSubsystem.INSTANCE.toNeutral,
//                outtakeSubsystem.INSTANCE.newClose,
                new Delay(0.1),
                transferSubsystem.INSTANCE.toOuttake,
                new Delay(0.2),
                transferSubsystem.INSTANCE.toNeutral,
                intakeSubsystem.INSTANCE.eat,
                new Delay(0.5),
                intakeSubsystem.INSTANCE.sleep,
                transferSubsystem.INSTANCE.toOuttake,
                new Delay(0.2),
                transferSubsystem.INSTANCE.toNeutral,
                hoodSubsystem.INSTANCE.far,
                intakeSubsystem.INSTANCE.eat,
                new Delay(0.7),
                intakeSubsystem.INSTANCE.sleep,
                transferSubsystem.INSTANCE.toOuttake,
                new Delay(0.2),
                outtakeSubsystem.INSTANCE.off(),
                transferSubsystem.INSTANCE.toNeutral
        ).requires(this);
    }

    public Command transpherSequencDiggerNiga() {
        return new SequentialGroup(
                multiFunctionSubsystem.INSTANCE.transpherSequencNiga(),
                intakeSubsystem.INSTANCE.eat,
                new Delay(0.3),
                intakeSubsystem.INSTANCE.sleep,
                multiFunctionSubsystem.INSTANCE.transpherSequencNiga(),
                intakeSubsystem.INSTANCE.eat,
                new Delay(0.3),
                intakeSubsystem.INSTANCE.sleep,
                multiFunctionSubsystem.INSTANCE.transpherSequencNiga()).requires(this);
    }

    public Command transferSequence() {
        return new SequentialGroup(
                transferSubsystem.INSTANCE.toOuttake,
                transferSubsystem.INSTANCE.toOuttake,
                transferSubsystem.INSTANCE.toOuttake,
                new Delay(0.2),
                transferSubsystem.INSTANCE.toNeutral,
                new Delay(0.1),
                intakeSubsystem.INSTANCE.eat,
                new Delay(0.6),
                transferSubsystem.INSTANCE.toOuttake,
                transferSubsystem.INSTANCE.toOuttake,
                transferSubsystem.INSTANCE.toOuttake,
                new Delay(0.2),
                transferSubsystem.INSTANCE.toNeutral,
                hoodSubsystem.INSTANCE.finalShot,
                new Delay(0.15),
                intakeSubsystem.INSTANCE.eat,
                new Delay(0.7),
                transferSubsystem.INSTANCE.toOuttake,
                transferSubsystem.INSTANCE.toOuttake,
                transferSubsystem.INSTANCE.toOuttake,
                new Delay(0.2),
                intakeSubsystem.INSTANCE.sleep,
                outtakeSubsystem.INSTANCE.off(),
                transferSubsystem.INSTANCE.toNeutral
        );
    }

    public Command outtakeGoonedSequence() {
        return new SequentialGroup(
                hoodSubsystem.INSTANCE.mid,
                transferSubsystem.INSTANCE.toNeutral,
                outtakeSubsystem.INSTANCE.mid(),
                new Delay (0.2),
                intakeSubsystem.INSTANCE.eat,
                new Delay (0.3),
                transferSubsystem.INSTANCE.toOuttake,
                new Delay(0.2),
                transferSubsystem.INSTANCE.toNeutral)
                .requires(this);
    }

    public Command transpherSequencNiga() {
        return new SequentialGroup(
                transferSubsystem.INSTANCE.toNeutral,
                transferSubsystem.INSTANCE.toOuttake,
                new Delay(.2),
                transferSubsystem.INSTANCE.toNeutral)
                .requires(this);
    }

    public Command stopPlease() {
        return new SequentialGroup(
                outtakeSubsystem.INSTANCE.off(),
                outtakeSubsystem.INSTANCE.noPower()
        ).requires(this);
    }

    public Command autonomousRoutine() {
        return new ParallelGroup(
                hoodSubsystem.INSTANCE.far,
                outtakeSubsystem.INSTANCE.far()
        ).requires(this);
    }

    public Command outtakeSequenceFar() {
        return new SequentialGroup(
                hoodSubsystem.INSTANCE.farDiddy,
                transferSubsystem.INSTANCE.toNeutral,
                outtakeSubsystem.INSTANCE.far(),
                new Delay(.67),
                transferSequenceFar(),
                outtakeSubsystem.INSTANCE.off()
        ).requires(this);

    }

    public Command transferSequenceFar() {
        return new SequentialGroup(
                new Delay(0.41),
                transferSubsystem.INSTANCE.toOuttake,
                transferSubsystem.INSTANCE.toOuttake,
                transferSubsystem.INSTANCE.toOuttake,
                new Delay(0.2),
                transferSubsystem.INSTANCE.toNeutral,
                new Delay(0.1),
                intakeSubsystem.INSTANCE.eat,
                new Delay(1),
                transferSubsystem.INSTANCE.toOuttake,
                transferSubsystem.INSTANCE.toOuttake,
                transferSubsystem.INSTANCE.toOuttake,
                new Delay(0.2),
                transferSubsystem.INSTANCE.toNeutral,
                new Delay(0.15),
                intakeSubsystem.INSTANCE.eat,
                new Delay(1),
                transferSubsystem.INSTANCE.toOuttake,
                transferSubsystem.INSTANCE.toOuttake,
                transferSubsystem.INSTANCE.toOuttake,
                new Delay(0.2),
                transferSubsystem.INSTANCE.toOuttake,
                transferSubsystem.INSTANCE.toOuttake,
                transferSubsystem.INSTANCE.toOuttake,
                new Delay(0.2),
                transferSubsystem.INSTANCE.toNeutral,
                new Delay(0.1),
                intakeSubsystem.INSTANCE.eat,
                new Delay(1),
                transferSubsystem.INSTANCE.toOuttake,
                transferSubsystem.INSTANCE.toOuttake,
                transferSubsystem.INSTANCE.toOuttake,
                new Delay(0.2),
                transferSubsystem.INSTANCE.toNeutral,
                new Delay(0.15),
                intakeSubsystem.INSTANCE.eat,
                new Delay(1),
                transferSubsystem.INSTANCE.toOuttake,
                transferSubsystem.INSTANCE.toOuttake,
                transferSubsystem.INSTANCE.toOuttake,
                new Delay(0.2),
                intakeSubsystem.INSTANCE.sleep,
                outtakeSubsystem.INSTANCE.off(),
                transferSubsystem.INSTANCE.toNeutral
        );
    }


    @Override
    public void initialize (){

    }

    @Override
    public void periodic (){
    }

}
