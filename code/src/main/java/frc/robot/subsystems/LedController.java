package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedController extends SubsystemBase {
    private boolean duplicateOnShuffleboard = false;
    private SimpleWidget widget;
    private GenericEntry entry;

    public interface PatternType {
        public double getValue();
    }
    public enum StripType {
        Adressable,
        Solid
    }
    public enum FixedPallatePatternType implements PatternType {
        Rainbow(-0.99),
        RainbowParty(-0.97),
        RainbowOcean(-0.95),
        RainbowLava(-0.93),
        RainbowForest(-0.91),
        RainbowGlitter(-0.89),
        Confetti(-0.87),
        ShotRed(-0.85),
        ShotBlue(-0.83),
        ShotWhite(-0.81),
        SinelonRainbow(-0.79),
        SinelonParty(-0.77),
        SinelonOcean(-0.75),
        SinelonLava(-0.73),
        SinelonForest(-0.71),
        BeatsPerMinuteRainbow(-0.69),
        BeatsPerMinuteParty(-0.67),
        BeatsPerMinuteOcean(-0.65),
        BeatsPerMinuteLava(-0.63),
        BeatsPerMinuteForest(-0.61),
        FireMedium(-0.59),
        FireLarge(-0.57),
        TwinklesRainbow(-0.55),
        TwinklesParty(-0.53),
        TwinklesOcean(-0.51),
        TwinklesLava(-0.49),
        TwinklesForest(-0.47),
        ColorWavesRainbow(-0.45),
        ColorWavesParty(-0.43),
        ColorWavesOcean(-0.41),
        ColorWavesLava(-0.39),
        ColorWavesForest(-0.37),
        LarsonScannerRed(-0.35),
        LarsonScannerGray(-0.33),
        LightChaseRed(-0.31),
        LightChaseBlue(-0.29),
        LightChaseGray(-0.27),
        HeartbeatRed(-0.25),
        HeartbeatBlue(-0.23),
        HeartbeatWhite(-0.21),
        HeartbeatGray(-0.19),
        BreathRed(-0.17),
        BreathBlue(-0.15),
        BreathGray(-0.13),
        StrobeRed(-0.11),
        StrobeBlue(-0.09),
        StrobeGold(-0.07),
        StrobeWhite(-0.05);
        
        private final double value;

        FixedPallatePatternType(double value) {
            this.value = value;
        }
        public double getValue() {
            return value;
        }
    }
    public enum Color1PatternType implements PatternType {
        EndToEndBlendToBlack(-0.03),
        LarsonScanner(-0.01),
        LightChase(0.01),
        HeartbeatSlow(0.03),    
        HeartbeatMedium(0.05),
        HeartbeatFast(0.07),
        BreathSlow(0.09),
        BreathFast(0.11),
        Shot(0.13),
        Strobe(0.15);

        private final double value;

        Color1PatternType(double value) {
            this.value = value;
        }
        public double getValue() {
            return value;
        }
    }
    public enum Color2PatternType implements PatternType {
        EndToEndBlendToBlack(0.17),
        LarsonScanner(0.19),
        LightChase(0.21),
        HeartbeatSlow(0.23),    
        HeartbeatMedium(0.25),
        HeartbeatFast(0.27),
        BreathSlow(0.29),
        BreathFast(0.31),
        Shot(0.33),
        Strobe(0.35);

        private final double value;

        Color2PatternType(double value) {
            this.value = value;
        }
        public double getValue() {
            return value;
        }
    }
    public enum Color1And2PatternType implements PatternType {
        Sparkle1On2(0.37),
        Sparkle2On1(0.39),
        Gradient1And2(0.41),
        BeatsPerMinute1And2(0.43),
        EndToEndBlend1To2(0.45),
        EndToEndBlend(0.47),
        Color1And2NoBlend(0.49),
        Twinkles(0.51),
        ColorWaves(0.53),
        Sinelon1And2(0.55);

        private final double value;

        Color1And2PatternType(double value) {
            this.value = value;
        }
        public double getValue() {
            return value;
        }
    }
    public enum SolidColorType implements PatternType {
        HotPink(0.57),
        DarkRed(0.59),
        Red(0.61),
        RedOrange(0.63),
        Orange(0.65),
        Gold(0.67),
        Yellow(0.69),
        LawnGreen(0.71),
        Lime(0.73),
        DarkGreen(0.75),
        Green(0.77),
        BlueGreen(0.79),
        Aqua(0.81),
        SkyBlue(0.83),
        DarkBlue(0.85),
        Blue(0.87),
        BlueViolet(0.89),
        Violet(0.91),
        White(0.93),
        Gray(0.95),
        DarkGray(0.97),
        Black(0.99);

        private final double value;
        SolidColorType(double value) {
            this.value = value;
        }
        public double getValue() {
            return value;
        }
    }
    
    int portID;
    StripType ledStripType;
    Spark ledController;
    
    public LedController(int portID, StripType type) {
        this.portID = portID;
        this.ledStripType = type;
        this.ledController = new Spark(portID);
    }

    public LedController(int portID, StripType type, String tabName) {
        this.portID = portID;
        this.ledStripType = type;
        this.ledController = new Spark(portID);
        this.duplicateOnShuffleboard = true;
        this.widget = Shuffleboard.getTab("SmartDashboard")
            .add("Status", false)
            .withProperties(Map.of("colorWhenFalse", "black"));
        this.entry = widget.getEntry();
    }
    public void setPattern(PatternType type) {

        if (duplicateOnShuffleboard && type instanceof SolidColorType) {

            SolidColorType color = (SolidColorType) type;
            widget.withProperties(Map.of("colorWhenTrue", mapToColorName(color)));
            entry.setBoolean(true);

        }

        ledController.set(type.getValue());
    }

    public String mapToColorName(SolidColorType pattern) {
        switch (pattern) {
            case HotPink:
                return "Hot Pink";
            case DarkRed:
                return "Dark Red";
            case Red:
                return "Red";
            case RedOrange:
                return "Red Orange";
            case Orange:
                return "Orange";
            case Gold:
                return "Gold";
            case Yellow:
                return "Yellow";
            case LawnGreen:
                return "Lawn Green";
            case Lime:
                return "Lime";
            case DarkGreen:
                return "Dark Green";
            case Green:
                return "Green";
            case BlueGreen:
                return "Blue Green";
            case Aqua:
                return "Aqua";
            case SkyBlue:
                return "Sky Blue";
            case DarkBlue:
                return "Dark Blue";
            case Blue:
                return "Blue";
            case BlueViolet:
                return "Blue Violet";
            case Violet:
                return "Violet";
            case White:
                return "White";
            case Gray:
                return "Gray";
            case DarkGray:
                return "Dark Gray";
            case Black:
                return "Black";
            default:
                return "Black";
        }
    }
}