package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

public class SubServoTurret implements Subsystem {
    public static Pose BLUEGOAL = new Pose(136, 139, Math.toRadians(0));

    public double turret1Pos;
    public double turretsetpos;
    public double turretTargetAngle;
    public static final SubServoTurret INSTANCE = new SubServoTurret();
    private SubServoTurret(){}
    private ServoEx turret1 = new ServoEx("turret");
    private ServoEx turret2 = new ServoEx("turret2");
    public Command testing = new SetPosition(turret1, 0.865).requires(this);
    public Command testing2 = new SetPosition(turret1, 0.14).requires(this);
    public Command middle = new SetPosition(turret1, 0.502).requires(this);

    // Set to 180 only if the turret physically points BACKWARD at servo-center (SERVO_CENTER).
    // 0 = turret points along robot-forward at center. This replaces the old hidden -180 flip
    // that normalizeAngle() used to bake in.
    private static final double MOUNT_OFFSET_DEG = 0;
    private static final double SERVO_SLOPE  = -0.002014;
    private static final double SERVO_CENTER =  0.5023;

    public double calculate(Pose botPose){
        double Offset_x = -3 * Math.cos(botPose.getHeading());
        double Offset_y = -3 * Math.sin(botPose.getHeading());
        double TurretPosX = botPose.getX() + Offset_x;
        double TurretPosY = botPose.getY() + Offset_y;
        double dx = BLUEGOAL.getX() - TurretPosX;
        double dy = BLUEGOAL.getY() - TurretPosY;
        double fieldAngleToGoal = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeading = Math.toDegrees(botPose.getHeading());
        // Relative bearing from robot-forward to the goal, wrapped to [-180, 180].
        // Stored in turretTargetAngle so getTurretTargetAngle() reflects the ACTUAL drive value.
        turretTargetAngle = wrap(fieldAngleToGoal - robotHeading + MOUNT_OFFSET_DEG);
        double desiredturredpos = SERVO_SLOPE * turretTargetAngle + SERVO_CENTER;
        return desiredturredpos;
        // right limit : 1
        // left limit : 0
        // servo turns in same direction as turret

    }
    static double wrap(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
    public void setPos(double servo1pos){
        turretsetpos = servo1pos;
    }
    public double getTurretTargetAngle(){
        return turretTargetAngle;
    }

    @Override
    public void initialize() {
        // initialization logic (runs on init)
    }
    @Override
    public void periodic(){
        turret1.setPosition(turretsetpos);
        turret1Pos = turret1.getPosition();
        //turret1.setPosition(turret1Pos);
        turret2.setPosition(turret1Pos);
        // periodic logic (runs every loop)
    }
}
