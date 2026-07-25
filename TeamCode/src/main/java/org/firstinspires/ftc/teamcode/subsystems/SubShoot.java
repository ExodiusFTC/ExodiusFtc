package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;


import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.MotorGroup;

import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;

import dev.nextftc.hardware.powerable.SetPower;
import com.seattlesolvers.solverslib.util.InterpLUT;


@Config
public class SubShoot implements Subsystem {
    public static final SubShoot INSTANCE = new SubShoot();
    InterpLUT lut = new InterpLUT();

    private SubShoot(){}


    private MotorEx shooterMotor = new MotorEx("SH");
    private MotorEx shooterMotor2 = new MotorEx("SH2");
    private MotorGroup SHOOTERS = new MotorGroup(shooterMotor, shooterMotor2);
    public boolean PIDTRUE;
    double shottune;
    double hoodtune;
    double goalDist;

    private ControlSystem controlSystem = ControlSystem.builder()
            .velPid(0.002, 0, 0)
            .basicFF(0.00036, 0, 0.14)
            .build();


    public Command StopShoot = new SetPower(shooterMotor, 0).requires(this);

    public Command PIDshot = new RunToVelocity(controlSystem, 1100, 30).requires(this);
    public Command PIDfarShot = new RunToVelocity(controlSystem, 1400, 30).requires(this);

    public Command InterpolationTuning(){
        return new RunToVelocity(controlSystem, shottune, 30 ).requires(this);
    }


    public double getvel(){
        return SHOOTERS.getVelocity();
    }
    public void setTargetvelocity(double targvel){

        shottune = targvel;
    }
    public double getTargetvelocity(){

        return shottune;
    }
    public void setDist(double dist){
        goalDist = dist;
    }






    @Override
    public void initialize() {
        lut.add(1.1, 0.2);
        lut.add(2.7, .5);
        lut.add(3.6, 0.75);
        lut.add(4.1, 0.9);
        lut.add(5, 1);
        lut.createLUT();
        // initialization logic (runs on init)

        //shooterMotor.getMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    @Override
    public void periodic() {
        double interVel = lut.get(goalDist);
        // periodic logic (runs every loop)
        if (PIDTRUE){
            SHOOTERS.setPower(controlSystem.calculate(SHOOTERS.getState()));
            //shooterMotor.setPower(controlSystem.calculate(shooterMotor.getState()));
        }
        if (!PIDTRUE){
            SHOOTERS.setPower(0);
        }


    }
    public void setPIDTRUE(boolean pidstate){
        PIDTRUE = pidstate;
    }
    public boolean getPIDTRUE(){
        return PIDTRUE;
    }
}