package org.firstinspires.ftc.teamcode.OpModes.Auto.paths;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.pedro.Constants;

public class Paths1 {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(55, 6.7, 180);
    private final Pose firstCycle = poseFactory.of(9, 7.5, 180);
    private final Pose firstShootForSecondTipStart = poseFactory.of(9, 7.5, 180);
    private final Pose firstShootForSecondTip = poseFactory.of(49, 121, 90);
    private final Pose firstShootForSecondTipControl1 = poseFactory.of(24, 76, 0);
    private final Pose firstShootForSecondTipControl2 = poseFactory.of(33, 116, 0);
    private final Pose getFlowerOne = poseFactory.of(49, 129, 90);
    private final Pose back = poseFactory.of(49, 125, 90);
    private final Pose forth = poseFactory.of(49, 129, 90);

    private final Pose Back2 = poseFactory.of(53, 125, 90);



    private final Pose moveToFlower2Start = poseFactory.of(52, 123, 180);
    private final Pose moveToFlower2 = poseFactory.of(20, 50, 180);
    private final Pose moveToFlower2Control1 = poseFactory.of(15, 96, 0);
    private final Pose getFlower2 = poseFactory.of(13.5, 50, 180);
    private final Pose BackfromFlower2 = poseFactory.of(18, 50, 180);
    private final Pose ForthfromFlower2 = poseFactory.of(13.5, 50, 180);


    private final Pose shootAgainStart = poseFactory.of(14, 50, 180);
    private final Pose shootAgain = poseFactory.of(54, 127.9024, 180);
    private final Pose parkStart = poseFactory.of(54, 127.9024, 180);
    private final Pose park = poseFactory.of(11, 116.9113, 180);

    public Pose startPose() {
        return start;
    }

    public Path firstCycle() {
        return Paths.line(start, firstCycle).linear(start, firstCycle).with(Constants.foresightConfig.maxPathSpeed.at(0.4));
    }

    public Path firstShootForSecondTip() {
        return Paths.curve(firstShootForSecondTipStart, firstShootForSecondTipControl1, firstShootForSecondTipControl2, firstShootForSecondTip).linear(firstShootForSecondTipStart, firstShootForSecondTip);
    }

    public Path getFlowerOne() {
        return Paths.line(firstShootForSecondTip, getFlowerOne).tangent();
    }
    public Path Back1() {
        return Paths.line(getFlowerOne, back).reverseTangent();
    }
    public Path Forth1() {
        return Paths.line(back, forth).tangent();
    }
    public Path Back2() {
        return Paths.line(forth, Back2).linear(forth, Back2);
    }

    public Path moveToFlower2() {
        return Paths.curve(moveToFlower2Start, moveToFlower2Control1, moveToFlower2).linear(moveToFlower2Start, moveToFlower2);
    }

    public Path getFlower2() {
        return Paths.line(moveToFlower2, getFlower2).tangent();
    }
    public Path Back3() {
        return Paths.line(getFlower2, BackfromFlower2).reverseTangent();
    }
    public Path Forth3() {
        return Paths.line(BackfromFlower2, ForthfromFlower2).tangent();
    }

    public Path shootAgain() {
        return Paths.line(shootAgainStart, shootAgain).linear(shootAgainStart, shootAgain);
    }

    public Path park() {
        return Paths.line(parkStart, park).linear(parkStart, park);
    }
}

