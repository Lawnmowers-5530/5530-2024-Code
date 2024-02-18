// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import io.github.oblarg.oblog.Loggable;
import frc.lib.Shot;
import frc.lib.ShotCalculator;
import frc.lib.Vector2D;
import frc.lib.VectorOperator;
import frc.lib.module.distanceSensor2024.subsystems.DistanceSensor;
import frc.lib.module.launcher2024.structs.LauncherV2CreateInfo;
import frc.lib.module.launcher2024.subsystems.LauncherV2;
import frc.lib.module.launcher2024.subsystems.LoaderV2;
import frc.lib.module.launcherAngle2024.subsystems.LauncherAngle;
import frc.robot.subsystems.StaticLimeLight;
import frc.robot.subsystems.Pgyro;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

public class RobotContainer implements Loggable{
  private final Field2d field;
  private SendableChooser<Command> autoChooser;
  private Swerve swerve = new Swerve();
  //private final DistanceSensor distanceSensor;
  //private final LoaderV2 loader;
  //private final LauncherAngle launcherAngle;
//
  //private final LauncherV2CreateInfo createInfo;
  //private final LauncherV2 launcher;

  //private final Trigger validTarget;
  private DoubleSupplier shotAngleSupplier;
  private BooleanSupplier slowModeSupplier;

  private CommandXboxController driverController;

  public RobotContainer() {
    //distanceSensor = new DistanceSensor();
    //loader = new LoaderV2(Constants.LoaderConstants.loaderMotorPort, Constants.LoaderConstants.isReversed, distanceSensor);
    //launcherAngle = new LauncherAngle(Constants.LauncherAngleConstants.motorPort);
    //createInfo = new LauncherV2CreateInfo().withLeftMotorPort(Constants.LauncherConstants.leftMotorPort).withRightMotorPort(Constants.LauncherConstants.rightMotorPort).withkP(Constants.LauncherConstants.kP).withkI(Constants.LauncherConstants.kI).withkD(Constants.LauncherConstants.kD).withkF(Constants.LauncherConstants.kF);
    //launcher = new LauncherV2(createInfo);
    //validTarget = new Trigger(StaticLimeLight.validTargetSupp());

    NamedCommands.registerCommand("shoot", shootCommand);

    shotAngleSupplier = new DoubleSupplier() {
    @Override
    public double getAsDouble() {
      return ShotCalculator.angleToTarget(swerve.getPose());
    }
  };
    slowModeSupplier = new BooleanSupplier() {
    @Override
    public boolean getAsBoolean() {
      return driverController.a().getAsBoolean();
    };
  };

    driverController = new CommandXboxController(0);
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
  
    SmartDashboard.putData("Auton chooser", autoChooser);

    field = new Field2d();
        SmartDashboard.putData("Field", field);

        // Logging callback for current robot pose
        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            field.setRobotPose(pose);
        });

        // Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            field.getObject("target pose").setPose(pose);
        });

        // Logging callback for the active path, this is sent as a list of poses
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            // Do whatever you want with the poses here
            field.getObject("path").setPoses(poses);
        });
  }

  private final Command swerveCmd = new RunCommand(
    () -> {
      double y = MathUtil.applyDeadband(driverController.getLeftY(), 0.15);
      double x = MathUtil.applyDeadband(driverController.getLeftX(), 0.15);
      double w = MathUtil.applyDeadband(driverController.getRightX(), 0.15);

      Vector2D vector = new Vector2D(y, x, false);
      swerve.drive(vector, -w, true);

}, swerve
);

  private final Command swerveSlowCmd = new RunCommand(
    () -> {
      double y = MathUtil.applyDeadband(driverController.getLeftY(), 0.15)/2;
      double x = MathUtil.applyDeadband(driverController.getLeftX(), 0.15)/2;
      double w = MathUtil.applyDeadband(driverController.getRightX(), 0.15)/2;

      Vector2D vector = new Vector2D(y, x, false);
      swerve.drive(vector, -w, true);

}, swerve
);

private final Command swerveCommand = new ConditionalCommand(swerveCmd, swerveSlowCmd, driverController.a());

private final Command resetGyro = new RunCommand(
  () -> {
    Pgyro.zeroGyro();
  }, new Subsystem[]{}
  );

//private final Command loadCommand = new RunCommand(
//  () -> {
//    loader.runUntilBeamBreak(Constants.LoaderConstants.loaderSpeed, Constants.LoaderConstants.loaderCutoffDistance);
//  }, loader
//  );

private final Command shootCommand = new RunCommand(
  () -> {
    Pose2d currentPose = swerve.getPose();

    Vector2D robotRelativeVector = new Vector2D(swerve.getRobotRelativeSpeeds().vxMetersPerSecond, swerve.getRobotRelativeSpeeds().vyMetersPerSecond, false);
    System.out.println(robotRelativeVector);
    Vector2D robotVector = VectorOperator.rotateVector2D(robotRelativeVector, Pgyro.getRot());
    System.out.println(robotVector);
    double distToTarget = Constants.targetTranslation.getDistance(currentPose.getTranslation());
    System.out.println(distToTarget);
    double angleToTarget = shotAngleSupplier.getAsDouble();

    Shot shot = ShotCalculator.vecFinal(robotVector, distToTarget, angleToTarget);
    System.out.println(shot);

    //temp logging
    SmartDashboard.putNumber("distToTarget", distToTarget);
    SmartDashboard.putNumber("angleToTarget", angleToTarget);
    SmartDashboard.putString("currentPose", currentPose.toString());
    SmartDashboard.putString("robotVector", robotVector.toString());

    SmartDashboard.putString("shot", shot.toString());

    //shot.Shoot(launcher, launcherAngle);
  }, new Subsystem[]{}//new Subsystem[]{launcher, launcherAngle}
  );

    private Command ampScore = AutoBuilder.pathfindToPose(
      new Pose2d(14.5, 7.5, new Rotation2d(Math.PI/2)),
      new PathConstraints(4.1, 1, 2, 1)
    );

  private void configureBindings() {
    swerve.setDefaultCommand(swerveCommand);
    //driverController.x().whileTrue(loadCommand);
    driverController.y().whileTrue(resetGyro);
    driverController.b().whileTrue(ampScore);
  }
  
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}
