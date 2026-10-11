package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;


import dev.nextftc.hardware.RobotController;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.robot.Mechanism;

public class Drivetrain implements Mechanism {
    public Drivetrain(){
        frontLeft.setDirection(NextMotor.Direction.REVERSE);
        backLeft.setDirection(NextMotor.Direction.REVERSE);
        frontRight.setDirection(NextMotor.Direction.FORWARD);
        backRight.setDirection(NextMotor.Direction.FORWARD);
    }
    public final NextMotor frontLeft = new NextMotor("FL");
    public final NextMotor frontRight = new NextMotor("FR" );
    public final NextMotor backLeft = new NextMotor("BL");
    public final NextMotor backRight =  new NextMotor("BR" );




}