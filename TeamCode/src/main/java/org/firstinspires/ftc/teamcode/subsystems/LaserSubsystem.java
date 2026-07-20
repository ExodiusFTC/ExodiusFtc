package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class LaserSubsystem {
    private DigitalChannel laser;
    public boolean detected;
    private boolean lastState = false;
    private int ballCount = 0;
    ElapsedTime elapsedTime = new ElapsedTime();
    public LaserSubsystem(HardwareMap hardwareMap){
        laser = hardwareMap.get(DigitalChannel.class, "laser");
        laser.setMode(DigitalChannel.Mode.INPUT);
    }
    public void update(){
        detected = laser.getState();
        if (detected && !lastState){
            ballCount++;
        }
        lastState = detected;


        // if (detected){
        //     elapsedTime.reset();
        //     elapsedTime.startTime();
        // }
        // else{
        //     elapsedTime.reset();
        // }
    }
    public boolean getState(){
        return detected;
    }
//    public int getBallCount(){
//        return ballCount;
//    }
     public boolean threeBalls(){
         if (elapsedTime.seconds() > 1.0){
             return true;
         }
         return false;
     }
}