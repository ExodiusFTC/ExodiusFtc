package org.firstinspires.ftc.teamcode.mechanisms;

import static dev.nextftc.units.Units.RotationsPerMinute;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;

public class Shooter implements Mechanism {
    NextMotor shooterMotor = new NextMotor("SH");
    NextMotor counterMotor = new NextMotor("SH2");

    public static double kP = 0.0000009;
    public static double kS = 0.07;
    public static double kV = 0.00033;


    public static double kP2 = 0.0000006;
    public static double kS2 = 0.05;
    public static double kV2 = 0.00036;


    public Shooter(){
        shooterMotor.getVelocityConstants().setKP(kP);
        shooterMotor.getVelocityConstants().setKS(kS);
        shooterMotor.getVelocityConstants().setKV(kV);
        counterMotor.getVelocityConstants().setKP(kP2);
        counterMotor.getVelocityConstants().setKS(kS2);
        counterMotor.getVelocityConstants().setKV(kV2);
    }
    public void setTargetVelocity(double targetVel){
        shooterMotor.setVelocitySetpoint(RotationsPerMinute.of(targetVel));
    }
    public void setTargetVelocity2(double targetVel2){
        counterMotor.setVelocitySetpoint(RotationsPerMinute.of(targetVel2));
    }
    public NextMotor getShooterMotor(){
        return shooterMotor;
    }
    public NextMotor getCounterMotor(){
        return counterMotor;
    }

    @Override
    public void periodic(){
    }

}
