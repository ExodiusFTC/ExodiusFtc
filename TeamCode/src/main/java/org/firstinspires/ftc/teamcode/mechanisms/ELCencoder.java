package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.nextftc.robot.Mechanism;

public class ELCencoder implements Mechanism {
    private static final double CPR = 4000.0;
    private static final int CHANNEL = 4;

    private OctoQuad octoQuad;
    private double currentTicks;
    private double tickVelocity;

    public ELCencoder() {}

    public void init(HardwareMap hwMap) {
        this.octoQuad = hwMap.get(OctoQuad.class, "octoquad");
        reset();
    }

    public void reset() {
        if (octoQuad != null) {
            octoQuad.resetSinglePosition(CHANNEL);
        }
    }

    @Override
    public void periodic() {
        if (octoQuad == null) return;

        currentTicks = octoQuad.readSinglePosition(CHANNEL);
        tickVelocity = octoQuad.readSingleVelocity(CHANNEL);
    }

    public double getCurrentTicks() {
        return currentTicks;
    }

    public double getTickVelocity() {
        return tickVelocity;
    }
}