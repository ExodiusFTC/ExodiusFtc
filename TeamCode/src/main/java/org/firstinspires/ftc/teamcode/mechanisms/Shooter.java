package org.firstinspires.ftc.teamcode.mechanisms;

import static dev.nextftc.units.Units.RotationsPerMinute;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;

public class Shooter implements Mechanism {
    NextMotor shooterMotor = new NextMotor("SH");
    NextMotor counterMotor = new NextMotor("SH2");
    public static double kP = 0;
    public static double kS = 0;
    public static double kV = 0;


    public Shooter(){
        shooterMotor.getVelocityConstants().setKP(kP);
        shooterMotor.getVelocityConstants().setKS(kS);
        shooterMotor.getVelocityConstants().setKV(kV);
    }
    public void setTargetVelocity(double targetVel){
        shooterMotor.setVelocitySetpoint(RotationsPerMinute.of(targetVel));
    }
    public NextMotor getShooterMotor(){
        return shooterMotor;
    }
    @Override
    public void periodic(){
    }

}
