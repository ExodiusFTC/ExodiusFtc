package org.firstinspires.ftc.teamcode.mechanisms;

import static dev.nextftc.units.Units.RotationsPerMinute;
import static dev.nextftc.units.Units.getRadiansPerSecond;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;

public class Shooter implements Mechanism {
    NextMotor shooterMotor = new NextMotor("SH");
    NextMotor counterMotor = new NextMotor("SH2");

    public static double kP = 0.003;
    public static double kS = 0.1;
    public static double kV = 0.00033;

    public static double kP2 = 0.003;
    public static double kS2 = 0.;
    public static double kV2 = 0.00033;


    public Shooter(){
        shooterMotor.getVelocityConstants().setKP(kP);
        shooterMotor.getVelocityConstants().setKI(0);
        shooterMotor.getVelocityConstants().setKD(0);
        shooterMotor.getVelocityConstants().setKS(kS);
        shooterMotor.getVelocityConstants().setKV(kV);
        shooterMotor.getVelocityConstants().setKA(0);




        counterMotor.getVelocityConstants().setKP(kP2);
        counterMotor.getVelocityConstants().setKI(0);
        counterMotor.getVelocityConstants().setKD(0);
        counterMotor.getVelocityConstants().setKS(kS2);
        counterMotor.getVelocityConstants().setKV(kV2);
        counterMotor.getVelocityConstants().setKA(0);
    }
    public void setTargetVelocity(double targetVel){
        shooterMotor.setVelocitySetpoint(getRadiansPerSecond(targetVel));
    }
    public void setTargetVelocity2(double targetVel2){
        counterMotor.setVelocitySetpoint(getRadiansPerSecond(targetVel2));
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