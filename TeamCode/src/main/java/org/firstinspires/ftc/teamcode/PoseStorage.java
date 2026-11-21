package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public final class PoseStorage {
    private PoseStorage() {}

    public static void savePose(Pose pose) {
        OpMode.blackboard.put("endPose", new Pose(pose.x(), pose.y(), pose.heading()));
    }

    public static Pose loadPose(Pose fallback) {
        Object p = OpMode.blackboard.get("endPose");
        return (p instanceof Pose) ? (Pose) p : fallback;
    }

    public static void clearPose() {
        OpMode.blackboard.remove("endPose");
    }

    public static void saveAlliance(boolean isRed) {
        OpMode.blackboard.put("isRed", isRed);
    }

    public static boolean isRed() {
        return !Boolean.FALSE.equals(OpMode.blackboard.get("isRed"));
    }
}