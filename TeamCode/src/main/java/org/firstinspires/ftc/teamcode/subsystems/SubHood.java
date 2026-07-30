package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.util.InterpLUT;

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
    public Command autohood = new SetPosition(Hood, 0.22).requires(this);
    public Command autohood2 = new SetPosition(Hood, 0.5).requires(this);
    InterpLUT hoodlut = new InterpLUT();

    public void sethoodtune(double tunevalue){
        hoodtune = tunevalue;
    }
    public double getHoodlut(double distGoal){
        return hoodlut.get(distGoal);
    }
    public double getHoodtune(){
        return Hood.getPosition();
    }
    public void initLut(){
        hoodlut = new InterpLUT();   // reset: INSTANCE is a persistent singleton, so re-init must
        hoodlut.add(10, 0.15);                             // rebuild rather than append (else X values stop increasing -> crash)
        hoodlut.add(44, 0.15);
        hoodlut.add(60, 0.45);
        hoodlut.add(80, 0.55);
        hoodlut.add(96, 0.55);
        hoodlut.add(115, 0.6);
        hoodlut.add(125, 0.65);
        hoodlut.add(142, 0.7);
        hoodlut.add(158, 0.7);
        hoodlut.add(182, 0.8);
        hoodlut.createLUT();
    }
    @Override
    public void initialize() {


        Hood.setPosition(0.5);
        // initialization logic (runs on init)
    }
    @Override
    public void periodic(){

    }
}
