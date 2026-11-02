package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.OpModes.tuningconfig.TargetDerivatives;

import dev.nextftc.control.feedback.PIDController;
import dev.nextftc.control.feedforward.SimpleFeedforward;
import dev.nextftc.hardware.actuators.NextCRServo;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.units.Units;

public class TurretServos implements Mechanism {
    public static Pose botPose;
    public double Pos1;
    public double targPos;
    public double currentPos;
    double kP;
    double kD;
    double kV;
    double kA;
    double kS;
    TargetDerivatives targetDeriv = new TargetDerivatives();
    PIDController turretPID = new PIDController(kP, 0.0, kD);
    SimpleFeedforward turretFeedforward = new SimpleFeedforward(kS, kV, kA);

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
    public void setTargetPos(double target){
        targPos = target;
    }
    public void setCurrentPos(double POS){
        currentPos = POS;
    }
    public void setBotPose(Pose robotposition){
        botPose = robotposition;
    }

    @Override
    public void periodic() {
        targetDeriv.update(targPos);
        double targVel   = targetDeriv.getVelocity();
        double targAccel = targetDeriv.getAcceleration();
        Pos1 = currentPos;
        turretServo2.setPower(turretServo1.getPower());
        double feedback = turretPID.calculate(targPos -currentPos);
        double feedforward = turretFeedforward.calculate(targVel, targAccel);
    }
}
