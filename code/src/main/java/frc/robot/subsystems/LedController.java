package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedController extends SubsystemBase {
    public interface patternType {
        public double getValue();
    }
    public enum stripType {
        Adressable,
        Solid
    }
    public enum fixedPalattePatternType implements patternType {
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

        fixedPalattePatternType(double value) {
            this.value = value;
        }
        public double getValue() {
            return value;
        }
    }
    public enum color1PatternType implements patternType {
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

        color1PatternType(double value) {
            this.value = value;
        }
        public double getValue() {
            return value;
        }
    }
    public enum color2PatternType implements patternType {
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

        color2PatternType(double value) {
            this.value = value;
        }
        public double getValue() {
            return value;
        }
    }
    public enum color1And2PatternType implements patternType {
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

        color1And2PatternType(double value) {
            this.value = value;
        }
        public double getValue() {
            return value;
        }
    }
    public enum solidColorType implements patternType {
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
        solidColorType(double value) {
            this.value = value;
        }
        public double getValue() {
            return value;
        }
    }
    
    int portID;
    stripType ledStripType;
    Spark ledController;
    
    public LedController(int portID, stripType type) {
        this.portID = portID;
        this.ledStripType = type;
        this.ledController = new Spark(portID);
    }
    public void setPattern(patternType type) {
        ledController.set(type.getValue());
    }
}