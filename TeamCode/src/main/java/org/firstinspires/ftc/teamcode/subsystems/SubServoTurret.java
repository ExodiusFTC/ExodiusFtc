package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

public class SubServoTurret implements Subsystem {
    public static Pose BLUEGOAL = new Pose(0, 144, Math.toRadians(0));

    public double turret1Pos;
    public double turretsetpos;
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

    public void setPos(double servo1pos){
        turretsetpos = servo1pos;
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
