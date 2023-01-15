package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.CT_LEDStrip;
import frc.robot.utils.CT_LEDStrip.ColorPattern;
import frc.robot.utils.CT_LEDStrip.GlowColor;
import frc.robot.utils.CT_LEDStrip.Speed;

// Here are some usage examples:
// RobotContainer.getLEDStripSubsystem().glow(GlowColor.Green);
// RobotContainer.getLEDStripSubsystem().snakeColors(Speed.Ludicrous, Color.kBlack, Color.kRed, Color.kBlue);
// RobotContainer.getLEDStripSubsystem().snakePattern(Speed.Ludicrous, Color.kBlack, ColorPattern.Cougartech);
// RobotContainer.getLEDStripSubsystem().rainbow();
// RobotContainer.getLEDStripSubsystem().moveColor(Speed.Ridiculous, Color.kBlue, Color.kRed);
// RobotContainer.getLEDStripSubsystem().moveColorPattern(Speed.Ridiculous, ColorPattern.SnakePacman);

public class LEDStripSubsystem extends SubsystemBase {
    
    private final static int LED_STRIP_PWM = 0;

    private static CT_LEDStrip m_ledStrip = new CT_LEDStrip(LED_STRIP_PWM);
    private StripState m_ledStripState;
    private Speed m_currentSpeed;
    private Color m_currentColorArray[];
    private ColorPattern m_currentColorPattern;
    private Color m_currentBackgroundColor;
    private GlowColor m_currentGlowColor;

    protected enum StripState {
        DoingNothing,
        MovingColor,
        MovingColorPattern,
        SnakingColors,
        SnakingPattern,
        Rainbow,
        Glowing
    }

    private void initStrip() {
        m_ledStrip.reset();
        m_ledStripState = StripState.DoingNothing;
        m_currentSpeed = Speed.Slow;
        m_currentColorPattern = ColorPattern.Cougartech;
        m_currentBackgroundColor = Color.kBlack;
        m_currentGlowColor = GlowColor.Yellow;
    }

    public LEDStripSubsystem() {
        initStrip();
    }

    public void resetStrip() {
        m_ledStrip.setColor(Color.kBlack);
        m_ledStripState = StripState.DoingNothing;
    }

    public void glow(GlowColor glowColor) {
        m_currentGlowColor = glowColor;
        m_ledStripState = StripState.Glowing;
    }

    public void snakeColors(Speed speed, Color color, Color... colorArray) {
        m_currentSpeed = speed;
        m_currentBackgroundColor = color;
        m_currentColorArray = colorArray;
        m_ledStripState = StripState.SnakingColors;
    }
    
    public void snakePattern(Speed speed, Color color, ColorPattern colorPattern) {
        m_currentSpeed = speed;
        m_currentBackgroundColor = color;
        m_currentColorPattern = colorPattern;
        m_ledStripState = StripState.SnakingPattern;
    }

    public void moveColor(Speed speed, Color... colorArray) {
        m_currentSpeed = speed;
        m_currentColorArray = colorArray;
        m_ledStripState = StripState.MovingColor;
    }

    public void moveColorPattern(Speed speed, ColorPattern colorPattern) {
        m_currentSpeed = speed;
        m_currentColorPattern = colorPattern;
        m_ledStripState = StripState.MovingColorPattern;
    }

    public void rainbow() {
        m_ledStripState = StripState.Rainbow;
    }

    @Override
    public void periodic() {
        switch (m_ledStripState) {
            case DoingNothing: {
                initStrip();
            }
            break;
            case MovingColor: {
                m_ledStrip.doMovingColors(m_currentSpeed, m_currentColorArray);
            }
            break;
            case MovingColorPattern: {
                m_ledStrip.doMovingColors(m_currentSpeed, m_currentColorPattern);
            }
            break;
            case SnakingColors: {
                m_ledStrip.doSnake(m_currentSpeed, m_currentBackgroundColor, m_currentColorArray);
            }
            break;           
            case SnakingPattern: {
                m_ledStrip.doSnake(m_currentSpeed, m_currentBackgroundColor, m_currentColorPattern);
            }
            break;
            case Rainbow: {
                m_ledStrip.doRainbow();
            }
            break;
            case Glowing: {
                m_ledStrip.doGlow(m_currentGlowColor);
            }
            break;
            default:
                initStrip();
        }        
    }
}
