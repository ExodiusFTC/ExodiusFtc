package org.firstinspires.ftc.teamcode.OpModes.Auto.paths;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public class HyperionPaths {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(55, 6.7, 180);
    private final Pose getFirst4 = poseFactory.of(9, 7.5, 180);
    private final Pose shootFirst4 = poseFactory.of(23, 127, -95.9725);
    private final Pose moveToFlowerStart = poseFactory.of(21.5019, 127, 180);
    private final Pose moveToFlower = poseFactory.of(20, 48, 180);
    private final Pose moveToFlowerControl1 = poseFactory.of(37, 40, 0);
    private final Pose goToFlower = poseFactory.of(15, 48, 180);
    private final Pose back = poseFactory.of(18, 48, 180);
    private final Pose finalShootStart = poseFactory.of(18, 48, 180);
    private final Pose finalShoot = poseFactory.of(52, 15, 90);
    private final Pose park = poseFactory.of(9, 99.5, 116.9704);

    public Path getFirst4() {
        return Paths.line(start, getFirst4).linear(start, getFirst4);
    }

    public Path shootFirst4() {
        return Paths.line(getFirst4, shootFirst4).reverseTangent();
    }

    public Path moveToFlower() {
        return Paths.curve(moveToFlowerStart, moveToFlowerControl1, moveToFlower).linear(moveToFlowerStart, moveToFlower);
    }

    public Path goToFlower() {
        return Paths.line(moveToFlower, goToFlower).tangent();
    }

    public Path back() {
        return Paths.line(goToFlower, back).constant(back);
    }

    public Path finalShoot() {
        return Paths.line(finalShootStart, finalShoot).linear(finalShootStart, finalShoot);
    }

    public Path park() {
        return Paths.line(finalShoot, park).tangent();
    }
}