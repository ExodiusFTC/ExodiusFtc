package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;

import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.robot.Mechanism;

public class TriangleIntake implements Mechanism {
    NextServo triangleServoR = new NextServo("tSR");
    NextServo triangleServoL = new NextServo("tSL");

    public TriangleIntake(){

    }
    public Command down(){
        return instant(() -> triangleServoR.setPosition(0.35)).requiring(triangleServoR);
    }
    public Command up(){
        return instant(() -> triangleServoR.setPosition(0.7)).requiring(triangleServoR);
    }
    @Override
    public void periodic() {
        triangleServoL.setPosition(1 - triangleServoR.getPosition());

    }
}
