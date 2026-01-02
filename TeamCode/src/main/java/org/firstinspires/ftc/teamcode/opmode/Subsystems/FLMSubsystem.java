package org.firstinspires.ftc.teamcode.opmode.Subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class FLMSubsystem implements Subsystem {
    public static final FLMSubsystem INSTANCE = new FLMSubsystem();
    private FLMSubsystem(){ }
    private MotorEx flm = new MotorEx("leftFront");
    public Command gooo = new SetPower(flm, 1).requires(this);
}
