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
    public static final SubServoTurret INSTANCE = new SubServoTurret();
    private SubServoTurret(){}
    private ServoEx turret1 = new ServoEx("turret");
    private ServoEx turret2 = new ServoEx("turret2");
    public Command testing = new SetPosition(turret1, 1.0).requires(this);
    public Command testing2 = new SetPosition(turret1, 0.0).requires(this);


    public double calculate(Pose botPose){
        double Offset_x = -3 * Math.cos(botPose.getHeading());
        double Offset_y = -3 * Math.sin(botPose.getHeading());
        double TurretPosX = botPose.getX() + Offset_x;
        double TurretPosY = botPose.getY() + Offset_y;
        double dx = BLUEGOAL.getX() - TurretPosX;
        double dy = BLUEGOAL.getY() - TurretPosY;
        double fieldAngleToGoal = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeading = Math.toDegrees(botPose.getHeading());
        double turretTargetAngle = fieldAngleToGoal - robotHeading;
        double CorrectTurning = normalizeAngle(turretTargetAngle);
        double desiredturredpos = 1.0/360.0  * CorrectTurning + 0.5;
        return desiredturredpos;
        // right limit : 1
        // left limit : 0
        // servo turns in same direction as turret

    }
    double normalizeAngle(double angle) {
        angle = -1 * (180 - angle);
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
    public void setPos(double servo1pos){
        turret1Pos = servo1pos;
    }

    @Override
    public void initialize() {
        // initialization logic (runs on init)
    }
    @Override
    public void periodic(){
        turret1Pos = turret1.getPosition();
        //turret1.setPosition(turret1Pos);
        turret2.setPosition(1 - turret1Pos);
        // periodic logic (runs every loop)
    }
}
