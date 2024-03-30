package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.LedController;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.LedController.FixedPalletePatternType;
import frc.robot.subsystems.LedController.PatternType;
import frc.robot.subsystems.LedController.SolidColorType;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PIDConstants;

public final class Constants {
    public static final boolean DEBUG_LOGGING = true;

    public static final double trackWidth = Units.inchesToMeters(24);
    public static final double wheelBase = Units.inchesToMeters(24);

    public static final double driveBaseRadius = Units
            .inchesToMeters(Math.sqrt(trackWidth * trackWidth + wheelBase * wheelBase));

    public static final class Mod0 { // FL
        public static final int driveMotor = 5;
        public static final int turnMotor = 6;
        public static final int canCoder = 13;
        public static final double angleOffset = 0;
    }

    public static final class Mod1 { // FR
        public static final int driveMotor = 7;
        public static final int turnMotor = 8;
        public static final int canCoder = 14;
        public static final double angleOffset = 13;
    }

    public static final class Mod3 { // RR
        public static final int driveMotor = 9;
        public static final int turnMotor = 10;
        public static final int canCoder = 15;
        public static final double angleOffset = 0;
    }

    public static final class Mod2 { // RL
        public static final int driveMotor = 11;
        public static final int turnMotor = 12;
        public static final int canCoder = 16;
        public static final double angleOffset = 0;
    }

    public static final class SwerveModuleConstants {
        public static final double conversionFactor = -1*(1/6.75)*Units.inchesToMeters(Math.PI*4);
        public static final class SwerveAnglePIDConstants {
            public static final double p = 0.0075;
            public static final double i = 0.0025;
            public static final double d = 0;
        }
    }

    public static Translation2d m0 = new Translation2d(Constants.trackWidth / 2, Constants.wheelBase / 2);
    public static Translation2d m1 = new Translation2d(Constants.trackWidth / 2, -Constants.wheelBase / 2);
    public static Translation2d m2 = new Translation2d(-Constants.trackWidth / 2, Constants.wheelBase / 2);
    public static Translation2d m3 = new Translation2d(-Constants.trackWidth / 2, -Constants.wheelBase / 2);

    public static final class Modules {
        public static final SwerveModule Mod_0 = new SwerveModule(Constants.Mod0.driveMotor, Constants.Mod0.turnMotor,
                Constants.Mod0.canCoder, Constants.Mod0.angleOffset);
        public static final SwerveModule Mod_1 = new SwerveModule(Constants.Mod1.driveMotor, Constants.Mod1.turnMotor,
                Constants.Mod1.canCoder, Constants.Mod1.angleOffset);
        public static final SwerveModule Mod_2 = new SwerveModule(Constants.Mod2.driveMotor, Constants.Mod2.turnMotor,
                Constants.Mod2.canCoder, Constants.Mod2.angleOffset);
        public static final SwerveModule Mod_3 = new SwerveModule(Constants.Mod3.driveMotor, Constants.Mod3.turnMotor,
                Constants.Mod3.canCoder, Constants.Mod3.angleOffset);
    }

    public static final class RotationConstants {
        public static final double kP = 0.1;
        public static final double kI = 0.1;
        public static final double kD = 0.1;
    }

    public static final class LoaderConstants {
        public static final int leftMotorPort = 20;
        public static final int rightMotorPort = 21;
        public static final boolean isReversed = true;
        public static final double loaderSpeed = 0.20;
        public static final double loaderCutoffDistance = 180;
        public static final int NOTE_LOADED_PRIORITY = 15;
    }

    public static final class LauncherConstants {
        public static final int leftMotorPort = 18;
        public static final int rightMotorPort = 19;
        public static final boolean isReversed = false;
        public static final double kP = 0.0002d;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = 0.00026;

        public static final double loaderShotSpeed = 0.35;

        public static final double launcherLowRevs = 600; // 800 // low speed for the launcher
        public static final double LAUNCHER_MED_REVS = 3000; // medium speed for the launcher
        public static final double launcherHighRevs = 4500; // high speed for the launcher

        public static final double launcherSpeedDiffPercent = 0.4;// was 0.2

        public static final int encoderCountsPerRev = 8192;
    }

    public static final class DumbLauncherAngleConstants {
        public static final double power = 0.2;
        public static final double atPositionMotorPowerMultiplier = 0.2;
    }
    public static final class CameraConstants {
        public static final String name = "fisheye";
        public static final int port = 0;
        public static final int width = 320;
        public static final int height = 240;
        public static final int fps = 300;
        public static final double exposure = 16;

    }

    public static final class LedControllerConstants {
        public static final int ledPort = 0;
        public static final LedController.StripType stripType = LedController.StripType.Adressable;
        public static final String shuffleBoardTabName = "competition";
    }

    public static final class LauncherAngleConstants {
        public static final int motorPort = 22;
        public static final boolean isReversed = false;

        public static final double kP = 0.01;
        public static final double kI = 0.00;
        public static final double kD = 0.00;

        public static final double conversionFactor = 5192 / 360;

        public static final double upPosition = 0.245;
        public static final double downPosition = 0.58;
        public static final double positionTolerance = 0.11;
    }

    public static final class IntakeConstants {
        public static final int motorPort = 24;
        public static final boolean isReversed = false;
        public static final double intakeSpeed = 0.5;
    }

    public static final class LauncherIntakeConstants {
        public static final int threshold = 90;
        public static final double speed = 0.175;
    }

    public static final class ClimberConstants {
        public static final int motorPort = 23;
        public static final boolean isReversed = true;

        public static final double speed = 1;
        public static final double maxHeight = 442.863220;
        public static final double minHeight = 0;

    }

    public static final class PathPlannerConstants {
        public static final PIDConstants translationConstants = new PIDConstants(4, 0, 0.0);
        public static final PIDConstants rotationConstants = new PIDConstants(3.5, 0.0, 0.0);// d was 0.2 and p was 6
                                                                                             // OTHER 2.4 p 0.05 d not
                                                                                             // working tho

        public static final PathConstraints constraints = new PathConstraints(
                3, 1.5, // linear
                180, 180 // angular
        );
    }

    public static final class AmpAssistConstants {
        public static int servoPort = 1;
        public static double up = 1.0; // 0.92
        public static double down = 0;
    }

    public static final class ExternalIntakeConstants {
        public static final int pivotMotorPort = 25;
        public static final int rollerMotorPort = 26;
        public static final double rollerSpeed = 0.35;

        public static final boolean isReversed = true;
        public static final double pivotDownPower = 0.15;
        public static final double pivotUpPower = -0.2;
        public static final double pivotUpPosition = 0.05;
        public static final double pivotDownPosition = 3.214283; // 3.214283
        public static final double pivotConversionFactor = (1 / 9) * (8 / 11); // gear ratio
    }

    public static final class LedConstants {
        public static final int groundIntakeAndAmpAnglePriority = 50;
        public static final PatternType groundIntakeAndAmpAnglePattern = SolidColorType.Red;

        public static final int intakeFromSourceReadyPriority = 40;
        public static final PatternType intakeFromSourceReadyPattern = SolidColorType.Blue;

        public static final int readyToShootPriority = 30;
        public static final PatternType readyToShootPattern = SolidColorType.Gold;

        public static final int noteLoadedPriority = 20;
        public static final PatternType noteLoadedPattern = SolidColorType.Green;

        public static final int slowModePriority = 10;
        public static final PatternType slowModePattern = SolidColorType.White;

        public static final int defaultPriority = 0;
        public static final PatternType defaultPattern = SolidColorType.LawnGreen;
    }

    public static final double shotDistance = 1;

    public static final double ejectSpeed = -0.3;

    public static SwerveDriveKinematics kinematics = new SwerveDriveKinematics(m0, m1, m2, m3);

    public static final Translation2d targetTranslation = new Translation2d(0, 5);

    public static final class Patterns {
        public static final PatternType NOTE_LOADED = SolidColorType.Blue;
        public static final PatternType NO_NOTE_ARM_READY = SolidColorType.Red;
        public static final PatternType NOTE_LOADED_IN_ZONE = SolidColorType.Green;
        public static final PatternType NO_NOTE_ARM_NOT_READY = FixedPalletePatternType.StrobeRed;
    }
}