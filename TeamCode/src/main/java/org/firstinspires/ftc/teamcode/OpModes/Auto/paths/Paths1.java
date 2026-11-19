package org.firstinspires.ftc.teamcode.OpModes.Auto.paths;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public class Paths1 {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8, 180);
    private final Pose firstCycle = poseFactory.of(10, 10, 180);
    private final Pose firstShootForSecondTipStart = poseFactory.of(6, 8, 180);
    private final Pose firstShootForSecondTip = poseFactory.of(47, 130, 90);
    private final Pose firstShootForSecondTipControl1 = poseFactory.of(24, 76, 0);
    private final Pose firstShootForSecondTipControl2 = poseFactory.of(33, 114, 0);
    private final Pose getFlowerOne = poseFactory.of(47, 133.5, 90);
    private final Pose moveToFlower2Start = poseFactory.of(47, 133.5, 90);
    private final Pose moveToFlower2 = poseFactory.of(16.5, 46.5, 180);
    private final Pose moveToFlower2Control1 = poseFactory.of(15, 96, 0);
    private final Pose getFlower2 = poseFactory.of(8, 46.5, 180);
    private final Pose shootAgainStart = poseFactory.of(8, 46.5, 180);
    private final Pose shootAgain = poseFactory.of(49.652, 127.9024, 180);
    private final Pose parkStart = poseFactory.of(49.652, 127.9024, 180);
    private final Pose park = poseFactory.of(4.1069, 116.9113, 180);

    public Pose startPose() {
        return start;
    }

    public Path firstCycle() {
        return Paths.line(start, firstCycle).linear(start, firstCycle);
    }

    public Path firstShootForSecondTip() {
        return Paths.curve(firstShootForSecondTipStart, firstShootForSecondTipControl1, firstShootForSecondTipControl2, firstShootForSecondTip).linear(firstShootForSecondTipStart, firstShootForSecondTip);
    }

    public Path getFlowerOne() {
        return Paths.line(firstShootForSecondTip, getFlowerOne).tangent();
    }

    public Path moveToFlower2() {
        return Paths.curve(moveToFlower2Start, moveToFlower2Control1, moveToFlower2).linear(moveToFlower2Start, moveToFlower2);
    }

    public Path getFlower2() {
        return Paths.line(moveToFlower2, getFlower2).tangent();
    }

    public Path shootAgain() {
        return Paths.line(shootAgainStart, shootAgain).linear(shootAgainStart, shootAgain);
    }

    public Path park() {
        return Paths.line(parkStart, park).linear(parkStart, park);
    }
}

