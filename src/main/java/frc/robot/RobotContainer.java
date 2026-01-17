// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Flywheel;

public class RobotContainer {
  private static final double kSimLoopPeriod = 0.004; // 4 ms
  private Notifier simNotifier = null;
  private double lastSimTime;

  @Logged private Flywheel flywheel = new Flywheel();

  private final CommandXboxController joystick = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(new InstantCommand());

    joystick.a().whileTrue(flywheel.runSlow());

    joystick.b().onTrue(flywheel.stopCommand());

    // configureSysidBindings(joystick);
  }

  public void configureSysidBindings(CommandXboxController joystick) {
    joystick.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
    joystick.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));

    /*
     * Joystick Y = quasistatic forward
     * Joystick A = quasistatic reverse
     * Joystick B = dynamic forward
     * Joystick X = dyanmic reverse
     */
    joystick.y().whileTrue(flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    joystick.a().whileTrue(flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    joystick.b().whileTrue(flywheel.sysIdDynamic(SysIdRoutine.Direction.kForward));
    joystick.x().whileTrue(flywheel.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  public void simulationInit() {
    lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - lastSimTime;
              lastSimTime = currentTime;

              /* Use the measured time delta, get battery voltage from WPILib */
              // drivetrain.updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
    simNotifier.startPeriodic(kSimLoopPeriod);
  }

  public void simulationPeriodic() {
    // Update drivetrain simulation
  }
}
