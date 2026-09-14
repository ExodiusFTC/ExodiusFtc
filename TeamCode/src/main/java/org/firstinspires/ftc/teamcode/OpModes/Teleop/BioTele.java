package org.firstinspires.ftc.teamcode.OpModes.Teleop;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;

import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.opmode.BulkReadHook;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;

@NextTeleop(name = "BioTele", group = "Teleop")
public class BioTele extends NextOpMode {
    private final Robot robot;
    public BioTele(Robot robot){
        super(robot, BulkReadHook.INSTANCE);
        this.robot = robot;
        CommandGamepad driver = new CommandGamepad(gamepad1);
        driver.a().onTrue(robot.intake.run());
        driver.b().onTrue(robot.intake.stop());

    }
    @Override
    public void start() {
        // Press 'A' to run the intake at full power

        // Press 'B' to stop the intake

    }



    @Override
    public void disabledPeriodic(){

    }
    @Override
    public void periodic(){
        if (gamepad1.aWasPressed()){
            robot.intake.run();
        }
        if (gamepad2.bWasPressed()){
            robot.intake.stop();
        }
        CommandGamepad driver = new CommandGamepad(gamepad1);
        driver.a().onTrue(robot.intake.run());
        driver.b().onTrue(robot.intake.stop());
        Telemetry.log("Status", "Running");
    }
}
