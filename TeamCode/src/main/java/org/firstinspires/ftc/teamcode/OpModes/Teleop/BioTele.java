package org.firstinspires.ftc.teamcode.OpModes.Teleop;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.TurretState;
import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;
import dev.nextftc.robot.triggers.Trigger;


@Config
@NextTeleop(name = "BioTele")
public class BioTele extends NextOpMode {
    public static Pose botPose;
    public static Pose defaultTelePose = new Pose(72,72,Math.toRadians(90));
    public static Pose startingPose = new Pose(10, 10, Math.toRadians(180));
    public static double targetVel = 580;

    public static double targetVel2 = -580;
    private final Robot robot;
    private Pose loadedPose;
    private boolean wasAiming = false;
    private boolean angleApplied = false;

    public BioTele(Robot robot){
        super(robot);
        this.robot = robot;
        telemetry.setMsTransmissionInterval(10);
        Telemetry.addBackend(FtcDashboard.getInstance().getTelemetry());
        Scheduler.reset();
        robot.intakeUp().schedule();

        // Load the pose auto saved; falls back to startingPose if auto didn't run (never null).
        loadedPose = PoseStorage.loadPose(startingPose);
        robot.getPedro().follower().setPose(loadedPose);

        // NOTE: turret angle is applied later in applyTurretAngle(), once hardware is live.
    }

    /** Applies the turret angle carried over from auton. Runs once, after hardware is ready. */
    private void applyTurretAngle() {
        if (angleApplied) return;
        if (TurretState.hasSaved) {
            robot.getIntake().setKnownAngle(TurretState.savedAngle);
        } else {
            robot.getIntake().resetEncoder();
        }
        angleApplied = true;
    }

    @Override
    public void start() {
        applyTurretAngle();   // backup in case INIT was skipped

        Trigger.Companion.getDefaultEventLoop().clear();

        CommandGamepad gp1 = new CommandGamepad(gamepad1);
        CommandGamepad gp2 = new CommandGamepad(gamepad2);


        gp1.rightBumper().onTrue(robot.getIntake().runFasst());
        gp1.rightBumper().onFalse(robot.getIntake().stop());

        gp1.x().onTrue(robot.intakeUp());
        gp1.y().onTrue(robot.intakeDown());
        //gp2.rightBumper().onTrue(robot.getTurretServos().run());
        gp2.rightBumper().onFalse(robot.getTurretServos().stop());
        gp2.leftBumper().onTrue(robot.getTurretServos().runOpposite());
        gp2.leftBumper().onFalse(robot.getTurretServos().stop());
        gp1.leftBumper().onTrue(robot.getBlocker().unblock().with(robot.runIntake()));
        gp1.leftBumper().onFalse(robot.getBlocker().block().with(robot.getIntake().stop()));
        robot.intakeUp();
    }



    @Override
    public void disabledPeriodic(){
        applyTurretAngle();   // runs once on the first INIT loop

        // Hold gamepad 2 X during INIT to force a re-zero (turret must be facing straight forward).
        if (gamepad2.x) {
            TurretState.clear();
            robot.getIntake().resetEncoder();
        }
        robot.intakeUp();

        Telemetry.log("Saved angle", TurretState.savedAngle);
        Telemetry.log("Has saved", TurretState.hasSaved);
        Telemetry.log("raw", robot.getIntake().getCurrentAngle());
        Telemetry.log("offset", robot.getIntake().offset);
        Telemetry.log("Actual angle", robot.getIntake().getActualAngle());
    }
    @Override
    public void periodic(){
        robot.getPedro().follower().manual(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        botPose = robot.getPedro().follower().pose();


        if(gamepad2.dpadDownWasPressed()){
            robot.getIntake().resetEncoder();
        }
        if(gamepad1.dpadDownWasPressed()){
            robot.getPedro().follower().setPose(startingPose);
        }
        double actualAngle = robot.getIntake().getActualAngle();
        TurretState.save(actualAngle);   // keep latest angle stored every loop
        robot.getTurretServos().setCurrentPos(actualAngle);
        double angle = robot.getTurretServos().calculateAimAngle(botPose.x(), botPose.y(), botPose.heading());

        double turretAngle = robot.getIntake().getCurrentAngle();
        double turretVel = robot.getIntake().getCurrentVel();
        Telemetry.log("Status", "Running");
        if(gamepad1.left_trigger >= 0.2){
            robot.getShooter().setTargetVelocity(targetVel);
            robot.getShooter().setTargetVelocity2(targetVel2);
        }
        else{
            robot.getShooter().setTargetVelocity(0);
            robot.getShooter().setTargetVelocity2(0);
        }

        // Hold gamepad 1 A to aim the turret at the goal; release to give it back to the bumpers.
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
    }

    @Override
    public void end(){
        // Clear so a later standalone tele run doesn't start from a stale auto pose.
        PoseStorage.clearPose();
    }
}