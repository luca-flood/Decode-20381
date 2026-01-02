package org.firstinspires.ftc.teamcode.opmode.Subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class FRMSubsystem implements Subsystem {
    public static final FRMSubsystem INSTANCE = new FRMSubsystem();
    private FRMSubsystem(){ }
    private MotorEx frm = new MotorEx("rightFront");
    public Command gooo = new SetPower(frm, 1).requires(this);
}
