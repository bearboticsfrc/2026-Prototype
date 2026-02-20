// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Elevator;

public class RobotContainer {
  private static final double kSimLoopPeriod = 0.004; // 4 ms
  private Notifier simNotifier = null;
  private double lastSimTime;

  // @Logged private Flywheel flywheel = new Flywheel();

  // @Logged private Turret turret = new Turret();

  private final CommandXboxController joystick = new CommandXboxController(0);

  @Logged private final Elevator elevator = new Elevator();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {

    // joystick.b().onTrue(elevator.goToSetpoint(() -> Elevator.Setpoint.Middle));
    // joystick.a().onTrue(elevator.goToSetpoint(() -> Elevator.Setpoint.Ground));
    // joystick.y().onTrue(elevator.goToSetpoint(() -> Elevator.Setpoint.Top));

    joystick.a().onTrue(elevator.goToSetpointAngle(() -> Rotations.of(0)));
    joystick.b().onTrue(elevator.goToSetpointAngle(() -> Rotations.of(.7)));
    joystick.y().onTrue(elevator.goToSetpointAngle(() -> Rotations.of(1.3)));

    elevator.setDefaultCommand(
        elevator.goToSetpointAngle(() -> Rotations.of(Math.abs(joystick.getRightY()))));

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(new InstantCommand());

    //  joystick.a().whileTrue(flywheel.runSlow());

    // joystick.b().onTrue(flywheel.stopCommand());

    // configureSysidBindings(joystick);

    //   joystick.a().onTrue(turret.setAngle(ninteyDegrees));
    //   joystick.b().onTrue(turret.setAngle(zeroDegrees));
    //   joystick.x().onTrue(turret.setAngle(oneEightyDegrees));
    //   joystick.y().onTrue(turret.setAngle(twoSeventyDegrees));

    //   rightStickActive()
    //       .whileTrue(
    //           turret.setAngle(
    //               () -> Radians.of(Math.atan2(-joystick.getRightX(), -joystick.getRightY()))));
  }

  public Trigger rightStickActive() {
    return new Trigger(
        () -> Math.abs(joystick.getRightY()) > 0.1 || Math.abs(joystick.getRightX()) > 0.1);
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

    // flywheel mode
    // joystick.y().whileTrue(flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // joystick.a().whileTrue(flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // joystick.b().whileTrue(flywheel.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // joystick.x().whileTrue(flywheel.sysIdDynamic(SysIdRoutine.Direction.kReverse));

  }

  public Angle ninteyDegrees = Degrees.of(90);
  public Angle oneEightyDegrees = Degrees.of(180);
  public Angle twoSeventyDegrees = Degrees.of(270);

  public Angle zeroDegrees = Degrees.of(0);

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
