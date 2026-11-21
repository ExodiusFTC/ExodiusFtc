package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.limelightvision.LLResult;

import dev.nextftc.hardware.webcams.NextLimelight;
import dev.nextftc.robot.Mechanism;

public class Limelight implements Mechanism {
    public boolean redTags35Visible;
    NextLimelight limelight = new NextLimelight("limelight");
    public Limelight(){
        limelight.startReading(9, 100);
    }
    @Override
    public void periodic(){
        LLResult result = limelight.getLatestResult();


    }
}
