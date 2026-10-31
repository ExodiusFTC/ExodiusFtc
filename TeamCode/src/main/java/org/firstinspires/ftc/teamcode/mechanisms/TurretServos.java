package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;

import dev.nextftc.hardware.actuators.NextCRServo;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.units.Units;

public class TurretServos implements Mechanism {

    NextCRServo turretServo1 = new NextCRServo("turretServo1");
    NextCRServo turretServo2 = new NextCRServo("turretServo2");
    public TurretServos(){

    }
    public Command run(){
        return instant(() -> turretServo1.setPower(1)).requiring(turretServo1);
    }
    public Command stop(){
        return instant(() -> turretServo1.setPower(0)).requiring(turretServo1);
    }
    public Command runOpposite(){
        return instant(() -> turretServo1.setPower(-1)).requiring(turretServo1);
    }
    @Override
    public void periodic() {
        turretServo2.setPower(turretServo1.getPower());
    }
}
