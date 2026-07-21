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
    public static final double DOWN_POS = 0.62;

    public Command RampDown = new SetPosition(RightRamp,DOWN_POS).requires(this);
    public Command RampUp = new SetPosition(RightRamp,UP_POS).requires(this);
    public Command Ramptune(double pos){
        return new SetPosition(RightRamp, pos).requires(this);
    }
    public double getUpPos(){
        return RightRamp.getPosition();
    }
    public double getDownPos(){
        return LeftRamp.getPosition();
    }


    @Override
    public void initialize(){
        RampDown.schedule();
    }
    @Override
    public void periodic(){

        LeftRamp.setPosition(1- RightRamp.getPosition());
    }



}