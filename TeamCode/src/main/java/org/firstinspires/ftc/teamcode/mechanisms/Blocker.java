package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;

import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.robot.Mechanism;

public class Blocker implements Mechanism {
    NextServo blocker = new NextServo("blocker");
    public Blocker() {

    }
    public Command unblock(){
        return instant(() -> blocker.setPosition(0.65)).requiring(blocker).requiring(blocker);
    }
    public Command block(){
        return instant(() -> blocker.setPosition(0.78)).requiring(blocker);
    }
    @Override
    public void periodic(){

    }
}
