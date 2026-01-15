package org.firstinspires.ftc.teamcode.opmode.Subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class BRMSubsystem implements Subsystem {
    public static final BRMSubsystem INSTANCE = new BRMSubsystem();
    private BRMSubsystem(){ }
    private MotorEx brm = new MotorEx("rightRear");
    public Command gooo = new SetPower(brm, 1).requires(this);
}
