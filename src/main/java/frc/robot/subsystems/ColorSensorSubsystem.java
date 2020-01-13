package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ColorSensor;

public class ColorSensorSubsystem extends SubsystemBase {
    
    private final ColorSensorV3 colorSensor = new ColorSensorV3(ColorSensor.PORT);
    private final ColorMatch colorMatcher = new ColorMatch();
    
    private PanelColor currentColor;

    public ColorSensorSubsystem() {
        for(PanelColor color : PanelColor.values()) {
            colorMatcher.addColorMatch(color.getColor());
        }
    }

    @Override
    public void periodic() {
        Color detectedColor = colorSensor.getColor();
        ColorMatchResult colorMatchResult = colorMatcher.matchClosestColor(detectedColor);
        for(PanelColor color : PanelColor.values()) {
            if(colorMatchResult.color == color.getColor()) {
                currentColor = color;
                break;
            }
        }
        SmartDashboard.putString("Color", currentColor.getName());
    }

    public PanelColor getCurrentColor() {
        return currentColor;
    }

    public enum PanelColor {
        RED     (ColorSensor.RED), // get colors from constants and make enums for each one
        GREEN   (ColorSensor.GREEN),
        BLUE    (ColorSensor.BLUE),
        YELLOW  (ColorSensor.YELLOW);

        private Color color;
        private String name;

        private PanelColor(double[] cmyk) {
            this.color = ColorMatch.makeColor(
                255 * ( 1 - cmyk[0] / 100 ) * ( 1 - cmyk[3] / 100 ),
                255 * ( 1 - cmyk[1] / 100 ) * ( 1 - cmyk[3] / 100 ),
                255 * ( 1 - cmyk[2] / 100 ) * ( 1 - cmyk[3] / 100 )
            );
            this.name = this.toString();
        }

        public Color getColor() {
            return color;
        }
        public String getName() {
            return name;
        }

        public static PanelColor fromChar(char c) {
            switch(c) {
                case 'R':
                    return RED;
                case 'G':
                    return GREEN;
                case 'B':
                    return BLUE;
                case 'Y':
                    return YELLOW;
                default:
                    return null;
            }
        }
    }
}