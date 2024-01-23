package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.SwerveModule;
import com.pathplanner.lib.util.PIDConstants;

public final class Constants {
    public static final double trackWidth = Units.inchesToMeters(24);
    public static final double wheelBase = Units.inchesToMeters(24);

    public static final double driveBaseRadius = Units.inchesToMeters(Math.sqrt(trackWidth*trackWidth + wheelBase*wheelBase));


    public static final class Mod0{ //FL
        public static final int driveMotor = 5;
        public static final int turnMotor = 6;
        public static final int canCoder = 13;
        public static final double angleOffset = 227;
    }
    public static final class Mod1{  //FR
        public static final int driveMotor = 7;
        public static final int turnMotor = 8;
        public static final int canCoder = 14;
        public static final double angleOffset = 11.5;
    }
    public static final class Mod2{ //RL
        public static final int driveMotor = 11;
        public static final int turnMotor = 12;
        public static final int canCoder = 16;
        public static final double angleOffset = 340;
    }
    public static final class Mod3{ //RR
        public static final int driveMotor = 9;
        public static final int turnMotor = 10;
        public static final int canCoder = 15;
        public static final double angleOffset = 56;
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
  
  
    public static SwerveDriveKinematics kinematics = new SwerveDriveKinematics(m0, m1, m2, m3);

    public static final PIDConstants translationConstants = new PIDConstants(0.5, 0.02, 0);
    public static final PIDConstants rotationConstants = new PIDConstants(0.65, 0.05, 0);

}