package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class LaserSubsystem {
    private DigitalChannel laser;
    public boolean detected;
    private boolean lastState = false;
    ElapsedTime elapsedTime = new ElapsedTime();
    public LaserSubsystem(HardwareMap hardwareMap){
        laser = hardwareMap.get(DigitalChannel.class, "laser");
        laser.setMode(DigitalChannel.Mode.INPUT);
    }
    public void update(){
        detected = laser.getState();
    }

    public boolean getDetection(){
        return detected;
    }

//    public int getBallCount(){
//        return ballCount;
//    }
     public boolean threeBalls(){
        if (detected){
            elapsedTime.startTime();
            if (elapsedTime.time()>=0.2){
                return true;
            }
            else {
                return false;
            }

        }
        else{
            elapsedTime.reset();
            return false;
        }
     };



}