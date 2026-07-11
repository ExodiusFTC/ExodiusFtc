package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

public class SubHood implements Subsystem {
    public static final SubHood INSTANCE = new SubHood();

    public double hoodtune;
    private SubHood(){}
    private ServoEx Hood = new ServoEx("hood");
    public Command HoodPosMiddle = new SetPosition(Hood, 0.5).requires(this);
    public Command HoodInterpolation(){
        return new SetPosition(Hood, hoodtune).requires(this);
    }
    public void sethoodtune(double tunevalue){
        hoodtune = tunevalue;
    }
    public double getHoodtune(){
        return hoodtune;
    }
    @Override
    public void initialize() {
        Hood.setPosition(0.35);
        // initialization logic (runs on init)
    }
    @Override
    public void periodic(){

    }
}
