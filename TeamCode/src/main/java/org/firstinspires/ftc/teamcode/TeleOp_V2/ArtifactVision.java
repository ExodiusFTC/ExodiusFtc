package org.firstinspires.ftc.teamcode.TeleOp_V2;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArtifactVision {

    private Limelight3A limelight;

    // --- TUNE THESE TOMORROW AT THE GARAGE ---
    private static final double kP = 0.03;
    private static final double TOLERANCE = 2.0;      // degrees, "close enough" to stop
    private static final double MIN_POWER = 0.25;      // floor so it doesn't crawl
    private static final double MAX_POWER = 0.6;       // ceiling so it doesn't overshoot

    public ArtifactVision(HardwareMap hardwareMap, int pipelineIndex) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(pipelineIndex);
        limelight.start();
    }

    /** Returns the latest result once per call, so we don't poll the Limelight twice per loop */
    private LLResult latest;

    public void update() {
        latest = limelight.getLatestResult();
    }

    public boolean hasTarget() {
        return latest != null && latest.isValid();
    }

    public double getTx() {
        if (hasTarget()) {
            return latest.getTx();
        } else {
            return 0;
        }
    }

    public double getTa() {
        if (hasTarget()) {
            return latest.getTa();
        } else {
            return 0;
        }
    }

    public boolean isAligned() {
        return hasTarget() && Math.abs(getTx()) < TOLERANCE;
    }

    /** Returns strafe power: positive/negative sign depends on your camera mount — verify tomorrow */
    public double getStrafePower() {
        if (!hasTarget()) return 0;

        double tx = getTx();
        if (Math.abs(tx) < TOLERANCE) return 0;

        double power = tx * kP;

        // apply floor so it doesn't crawl
        if (Math.abs(power) < MIN_POWER) {
            power = Math.copySign(MIN_POWER, power);
        }

        // clamp ceiling so it doesn't overshoot
        power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));

        return power;
    }
}