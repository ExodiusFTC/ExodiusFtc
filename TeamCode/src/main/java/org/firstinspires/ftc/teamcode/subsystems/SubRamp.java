package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;

public class SubRamp implements Subsystem {
    public static final SubRamp INSTANCE = new SubRamp();
    private SubRamp(){}
    private ServoEx RightRamp = new ServoEx("RRamp");
    private ServoEx LeftRamp = new ServoEx("LRamp");

    @Override
    public void initialize() {
        // initialization logic (runs on init)
    }
    @Override
    public void periodic(){
        // periodic logic (runs every loop)
    }

}
