package org.firstinspires.ftc.teamcode.opmode.Subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class intakeSubsystem implements Subsystem {

    public static final intakeSubsystem INSTANCE = new intakeSubsystem();
    private intakeSubsystem(){ }
    private MotorEx intake = new MotorEx("intake");
    public Command eat = new SetPower(intake, -1).requires(this);
    public Command slowSpit = new SetPower(intake, 0.5).requires(this);

    public Command spit = new SetPower(intake, 1).requires(this);

    public Command sleep = new SetPower(intake, 0).requires(this);
}
