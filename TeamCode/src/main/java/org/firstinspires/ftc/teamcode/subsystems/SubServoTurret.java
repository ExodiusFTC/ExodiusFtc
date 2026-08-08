package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;
import com.seattlesolvers.solverslib.util.InterpLUT;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

public class SubServoTurret implements Subsystem {
    public static Pose BLUEGOAL = new Pose(0, 144, Math.toRadians(0));
    public static Pose REDGOAL = new Pose(144, 142, Math.toRadians(180));

    public double turret1Pos;
    public double turretsetpos;
    public static final SubServoTurret INSTANCE = new SubServoTurret();
    private SubServoTurret(){}
    private ServoEx turret1 = new ServoEx("turret");
    private ServoEx turret2 = new ServoEx("turret2");
    public Command testing = new SetPosition(turret1, 0.865).requires(this); //+180
    public Command testing2 = new SetPosition(turret1, 0.14).requires(this);  //-180
    public Command middle = new SetPosition(turret1, 0.502).requires(this); //0
    InterpLUT turretLut = new InterpLUT();


    // Set to 180 only if the turret physically points BACKWARD at servo-center (SERVO_CENTER).
    // 0 = turret points along robot-forward at center. This replaces the old hidden -180 flip
    // that normalizeAngle() used to bake in.
    public double calculate(Pose botPose){
        double dx = BLUEGOAL.getX() - botPose.getX();
        double dy = BLUEGOAL.getY() - botPose.getY();
        double fieldAngleToGoal = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeading = Math.toDegrees(botPose.getHeading());
        double turretTargetAngle = fieldAngleToGoal - robotHeading;
        double CorrectTurning = normalizeAngle(turretTargetAngle);
        //double despos = 0.00201389*CorrectTurning+0.502333;
        return CorrectTurning;
    }

    public double calculateRed(Pose botPose){
        double dx = REDGOAL.getX() - botPose.getX();
        double dy = REDGOAL.getY() - botPose.getY();
        double fieldAngleToGoal = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeading = Math.toDegrees(botPose.getHeading());
        double turretTargetAngle = fieldAngleToGoal - robotHeading;
        double CorrectTurning = normalizeAngle(turretTargetAngle);
        return CorrectTurning;
    }

    public void setPos(double servo1pos){
        turretsetpos = turretLut.get(servo1pos);
    }
    double normalizeAngle(double angle) {
        angle = -1 * (180 - angle);
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
    public double getPos1(){
        return turret1.getPosition();
    }
    public double getPos2(){
        return turret2.getPosition();
    }
    public void initlut(){
        turretLut = new InterpLUT();   // reset: INSTANCE is a persistent singleton, so re-init must
        turretLut.add(-180, 0.14);
        turretLut.add(0, 0.502);
        turretLut.add(180, 0.867);
        turretLut.createLUT();
    }


    @Override
    public void initialize() {
    }
    @Override
    public void periodic(){
        turret1.setPosition(turretsetpos);
        turret1Pos = turret1.getPosition();
        //turret1.setPosition(turret1Pos);
        turret2.setPosition(turret1Pos);
    }
}