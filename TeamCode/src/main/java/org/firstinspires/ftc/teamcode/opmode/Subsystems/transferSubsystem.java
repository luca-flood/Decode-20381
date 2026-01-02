package org.firstinspires.ftc.teamcode.opmode.Subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

public class transferSubsystem implements Subsystem {

    public static final transferSubsystem INSTANCE = new transferSubsystem();
    private transferSubsystem(){ }
    private ServoEx transfer = new ServoEx("transfer");
    public Command toOuttake = new SetPosition(transfer, 0.9).requires(this);

    public Command toNeutral = new SetPosition(transfer, 0.68).requires(this);
    public Command transferSequence() {
        return new SequentialGroup(
                new Delay(0.2),
                transferSubsystem.INSTANCE.toOuttake,
                new Delay(0.05),
                transferSubsystem.INSTANCE.toNeutral,
                new Delay(0.1),
                transferSubsystem.INSTANCE.toOuttake,
                new Delay(0.05),
                transferSubsystem.INSTANCE.toNeutral,
                new Delay(0.1),
                transferSubsystem.INSTANCE.toOuttake,
                new Delay(0.05),
                intakeSubsystem.INSTANCE.sleep
        ).requires(this);
    }

    public double getJawn() {
        return transfer.getPosition();
    }
}

