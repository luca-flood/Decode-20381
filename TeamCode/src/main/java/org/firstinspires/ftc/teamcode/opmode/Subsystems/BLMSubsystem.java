package org.firstinspires.ftc.teamcode.opmode.Subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class BLMSubsystem implements Subsystem {
    public static final BLMSubsystem INSTANCE = new BLMSubsystem();
    private BLMSubsystem(){ }
    private MotorEx blm = new MotorEx("leftRear");
    public Command gooo = new SetPower(blm, 1).requires(this);
}
