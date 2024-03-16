package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
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

    public static final double driveBaseRadius = Units.inchesToMeters(Math.sqrt(trackWidth*trackWidth + wheelBase*wheelBase));


    public static final class Mod0{ //FL
        public static final int driveMotor = 5;
        public static final int turnMotor = 6;
        public static final int canCoder = 13;
        public static final double angleOffset = 0;
    }
    public static final class Mod1{ //FR
        public static final int driveMotor = 7;
        public static final int turnMotor = 8;
        public static final int canCoder = 14;
        public static final double angleOffset = 0;
    }
    public static final class Mod3{ //RR
        public static final int driveMotor = 9;
        public static final int turnMotor = 10;
        public static final int canCoder = 15;
        public static final double angleOffset = 0;
    }
    public static final class Mod2{ //RL
        public static final int driveMotor = 11;
        public static final int turnMotor = 12;
        public static final int canCoder = 16;
        public static final double angleOffset = 0;
    }
    public static Translation2d m0 = new Translation2d(Constants.trackWidth/2, Constants.wheelBase/2);
    public static Translation2d m1 = new Translation2d(Constants.trackWidth/2, -Constants.wheelBase/2);
    public static Translation2d m2 = new Translation2d(-Constants.trackWidth/2, Constants.wheelBase/2);
    public static Translation2d m3 = new Translation2d(-Constants.trackWidth/2, -Constants.wheelBase/2);

    public static final class Modules{
        public static final SwerveModule Mod_0 = new SwerveModule(Constants.Mod0.driveMotor, Constants.Mod0.turnMotor, Constants.Mod0.canCoder, Constants.Mod0.angleOffset);
        public static final SwerveModule Mod_1 = new SwerveModule(Constants.Mod1.driveMotor, Constants.Mod1.turnMotor, Constants.Mod1.canCoder, Constants.Mod1.angleOffset);
        public static final SwerveModule Mod_2 = new SwerveModule(Constants.Mod2.driveMotor, Constants.Mod2.turnMotor, Constants.Mod2.canCoder, Constants.Mod2.angleOffset);
        public static final SwerveModule Mod_3 = new SwerveModule(Constants.Mod3.driveMotor, Constants.Mod3.turnMotor, Constants.Mod3.canCoder, Constants.Mod3.angleOffset);
    }

    public static final class RotationConstants{
        public static final double kP = 0.1;
        public static final double kI = 0.1;
        public static final double kD = 0.1;
    }
    
    public static final class LoaderConstants{
        public static final int leftMotorPort = 20;
        public static final int rightMotorPort = 21;
        public static final boolean isReversed = true;
        public static final double loaderSpeed = 0.3;
        public static final double loaderCutoffDistance = 180;
        public static final int NOTE_LOADED_PRIORITY = 15;
    }

    public static final class LauncherConstants{
        public static final int leftMotorPort = 18;
        public static final int rightMotorPort = 19;
        public static final boolean isReversed = false;
        public static final double kP = 0.0002d;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = 0.00026;

        public static final double loaderShotSpeed = 0.35;

        public static final double LAUNCHER_LOW_REVS = 465; //1000 // low speed for the launcher
        public static final double LAUNCHER_MED_REVS = 3000; // medium speed for the launcher
        public static final double LAUNCHER_HIGH_REVS = 4500; // high speed for the launcher

        public static final double LAUNCHER_SPEED_DIFF_PERCENT = 0.4;//was 0.2
    }

    public static final class LauncherAngleConstants{
        public static final int motorPort = 22;
        public static final boolean isReversed = false;

        public static final double kP = 0.01;
        public static final double kI = 0.00;
        public static final double kD = 0.00;

        public static final double conversionFactor = 5192/360; //TODO: logically should be 1/360

        public static final double ampPosition = 0.191; // TODO: find the correct value
        public static final double speakerPosition = 0.665; // TODO: find the correct value
        public static final double positionTolerance = 0.08;
    }

    public static final class IntakeConstants{
        public static final int motorPort = 24;
        public static final boolean isReversed = false;
        public static final double intakeSpeed = 0.33;
    }

    public static final class LauncherIntakeConstants{
        public static final int threshold = 70;
        public static final double speed = 0.175;
    }

    public static final class ClimberConstants{
        public static final int motorPort = 23;
        public static final boolean isReversed = true;

        public static final double speed = 1;
        public static final double maxHeight = 442.863220;
        public static final double minHeight = 0;
        
    }

    public static final class PathPlannerConstants{
        public static final PIDConstants translationConstants = new PIDConstants(2, 0, 0.0);
    public static final PIDConstants rotationConstants = new PIDConstants(1.5, 0.0, 0);

    public static final PathConstraints constraints = new PathConstraints(
        3, 2, //linear
        360, 420 //angular
    );
    }

    public static final class LedConstants{
        public static final int groundIntakeAndAmpAnglePriority = 50;
        public static final PatternType groundIntakeAndAmpAnglePattern = SolidColorType.Red;

        public static final int intakeFromSourceReadyPriority = 40;
        public static final PatternType intakeFromSourceReadyPattern = SolidColorType.Aqua;

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