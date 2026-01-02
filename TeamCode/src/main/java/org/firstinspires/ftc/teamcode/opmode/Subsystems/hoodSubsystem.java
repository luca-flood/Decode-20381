package org.firstinspires.ftc.teamcode.opmode.Subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

public class hoodSubsystem implements Subsystem {

    public static final hoodSubsystem INSTANCE = new hoodSubsystem();
    private hoodSubsystem(){ }
    private ServoEx hood = new ServoEx("hood");
    public Command close = new SetPosition(hood, 0.08).requires(this);
    public Command closeMid = new SetPosition(hood, 0.55).requires(this);
    public Command finalShot = new SetPosition(hood, 0.6).requires(this);
    public Command mid = new SetPosition(hood, 0.45).requires(this);
    public Command far = new SetPosition(hood, 0.7).requires(this);
    public Command auto = new SetPosition(hood, 0.5).requires(this);
    public Command one = new SetPosition(hood, 1).requires(this);

    public Command lowest = new SetPosition(hood, 0.11).requires(this);

    public Command farDiddy = new SetPosition(hood, 0.575);

    public Command goon(double val) {
        return new SetPosition(hood, val);
    }

    public double getDaddy() {
        return hood.getPosition();
    }
}