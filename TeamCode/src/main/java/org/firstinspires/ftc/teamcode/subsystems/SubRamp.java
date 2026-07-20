package org.firstinspires.ftc.teamcode.subsystems;

import java.util.Set;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

public class SubRamp implements Subsystem {

    public static final SubRamp INSTANCE = new SubRamp();
    private SubRamp(){}

    private ServoEx RightRamp = new ServoEx("RRamp");
    private ServoEx LeftRamp = new ServoEx("LRamp");

    public static final double UP_POS = 0.5;
    public static final double DOWN_POS = 0.7;

    public Command RampDown = new SetPosition(RightRamp,1).requires(this);

    public Command setLeftRampPos(double pos) {
        return new InstantCommand(() -> LeftRamp.setPosition(pos));
    }
    @Override
    public void initialize(){}
    @Override
    public void periodic(){
        LeftRamp.setPosition(1- RightRamp.getPosition());
    }



}