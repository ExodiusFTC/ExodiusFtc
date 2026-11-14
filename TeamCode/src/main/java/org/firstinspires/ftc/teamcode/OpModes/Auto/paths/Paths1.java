package org.firstinspires.ftc.teamcode.OpModes.Auto.paths;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public class Paths1 {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 9, 90);
    private final Pose path1 = poseFactory.of(8, 10, 180);

    public Path path1() {
        return Paths.line(start, path1).constant(path1);
    }
}
