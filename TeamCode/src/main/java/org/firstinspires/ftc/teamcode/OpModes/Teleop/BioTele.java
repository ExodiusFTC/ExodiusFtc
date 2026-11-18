package org.firstinspires.ftc.teamcode.OpModes.Teleop;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Robot;
import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;
import dev.nextftc.robot.triggers.Trigger;


@Config
@NextTeleop(name = "BioTele")
public class BioTele extends NextOpMode {
    public static Pose botPose;
    public static Pose startingPose = new Pose(8, 136, Math.toRadians(180));
    public static double targetVel = 580;

    public static double targetVel2 = 600;
    private final Robot robot;
    private boolean wasAiming = false;
    public BioTele(Robot robot){
        super(robot);
        this.robot = robot;
        telemetry.setMsTransmissionInterval(10);
        //this.robot.initHardware(hardwareMap);
        Telemetry.addBackend(FtcDashboard.getInstance().getTelemetry());
        Scheduler.reset();
        robot.intakeUp();


    }
    @Override
    public void start() {

        Trigger.Companion.getDefaultEventLoop().clear();

        CommandGamepad gp1 = new CommandGamepad(gamepad1);
        CommandGamepad gp2 = new CommandGamepad(gamepad2);
        robot.getPedro().follower().setPose(startingPose);


        gp1.rightBumper().onTrue(robot.getIntake().run());
        gp1.rightBumper().onFalse(robot.getIntake().stop());

        gp1.x().onTrue(robot.intakeUp());
        gp1.y().onTrue(robot.intakeDown());
        gp2.rightBumper().onTrue(robot.getTurretServos().run());
        gp2.rightBumper().onFalse(robot.getTurretServos().stop());
        gp2.leftBumper().onTrue(robot.getTurretServos().runOpposite());
        gp2.leftBumper().onFalse(robot.getTurretServos().stop());
        robot.intakeUp();
        robot.getIntake().resetEncoder();










    }



    @Override
    public void disabledPeriodic(){
        robot.getIntake().resetEncoder();
        robot.intakeUp();
    }
    @Override
    public void periodic(){
        robot.getPedro().follower().manual(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        botPose = robot.getPedro().follower().pose();


        if(gamepad2.xWasPressed()){
            robot.getIntake().resetEncoder();
        }
        // current turret angle first, so the aim calculation uses this loop's reading
        double actualAngle = robot.getIntake().getActualAngle();
        robot.getTurretServos().setCurrentPos(actualAngle);
        double angle = robot.getTurretServos().calculateAimAngle(botPose.x(), botPose.y(), botPose.heading());

        double turretAngle = robot.getIntake().getCurrentAngle();
        double turretVel = robot.getIntake().getCurrentVel();
        Telemetry.log("Status", "Running");
        Telemetry.log("Status", "Running");
        if(gamepad1.left_trigger >= 0.2){
            robot.getShooter().setTargetVelocity(targetVel);
            robot.getShooter().setTargetVelocity2(targetVel2);
        }
        else{
            robot.getShooter().setTargetVelocity(0);
            robot.getShooter().setTargetVelocity2(0);
        }

        // Hold gamepad 2 A to aim the turret at the goal; release to give it back to the bumpers.
        if(gamepad1.a){
            robot.getTurretServos().setTargetPos(angle);
            robot.getTurretServos().setClosedLoop(true);
            wasAiming = true;
        }
        else if(wasAiming){
            robot.getTurretServos().setManualPower(0);   // closed loop off, turret stops
            wasAiming = false;
        }

        Telemetry.log("current vel", robot.getShooter().getShooterMotor().getEncoderVelocity());
        Telemetry.log("current vel 2", robot.getShooter().getCounterMotor().getEncoderVelocity());
        Telemetry.log("turretAngle", turretAngle);
        Telemetry.log("turretVel", turretVel);
        Telemetry.log("Actual angle", actualAngle);
        Telemetry.log("aim angle", angle);
        Telemetry.log("turret error", robot.getTurretServos().getError());
        Telemetry.log("Bot Pose", botPose.toString());
        Telemetry.log("heading", Math.toDegrees(botPose.heading()));

//        Telemetry.log("current angle", robot.getTurret().getCurrentAngle());
//        Telemetry.log("current vel", robot.getTurret().getCurrentVel());
//        Telemetry.log("Encoder Ticks", robot.getEncoder().getCurrentTicks());
//        Telemetry.log("Encoder Velocity", robot.getEncoder().getTickVelocity());
    }

    @Override
    public void end(){

    }
}