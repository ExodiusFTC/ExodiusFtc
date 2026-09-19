package org.firstinspires.ftc.teamcode.OpModes.Teleop;


import com.pedropathing.ivy.Scheduler;
import org.firstinspires.ftc.teamcode.Robot;
import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;
import dev.nextftc.robot.triggers.Trigger;

@NextTeleop(name = "BioTele")
public class BioTele extends NextOpMode {
    private final Robot robot;
    public BioTele(Robot robot){
        super(robot);
        this.robot = robot;
        Scheduler.reset();
    }
    @Override
    public void start() {
        Trigger.Companion.getDefaultEventLoop().clear();

        CommandGamepad gp1 = new CommandGamepad(gamepad1);
        CommandGamepad gp2 = new CommandGamepad(gamepad2);
        gp1.a().onTrue(robot.getIntake().run());
        gp1.b().onTrue(robot.getIntake().stop());

        // Press 'A' to run the intake at full power

        // Press 'B' to stop the intake

    }



    @Override
    public void disabledPeriodic(){

    }
    @Override
    public void periodic(){
        Telemetry.log("Status", "Running");
    }
    @Override
    public void end(){

    }
}
