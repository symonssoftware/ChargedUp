package frc.robot.utils;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.utils.CT_LEDStrip.GlowColor;
import frc.robot.utils.CT_LEDStrip.MovementType;
import frc.robot.utils.CT_LEDStrip.Speed;

public class CT_LEDStrip extends AddressableLED {

    private final static double GLOW_MAX = 1;
    private final static double GLOW_MIN = 0.05;
    /**
     * We want to increase by 8 out of 255, and as the color class
     * takes values from (0 - 1) instead of (0 - 255),
     * (0.0314 to 1) = (8 to 255)
     */
    private final static double INCREASE_VALUE = 0.0314;

    private AddressableLEDBuffer m_LEDBuffer;
    private int m_rainbowFirstPixelHue = 0;

    private int m_snakeLoopIndex = 0;
    private int m_snakeCounter = 0;

    private int m_movingColorsCounter = 0;
    private int m_movingColorIndex = 0;

    private int m_glowCounter = 0;
    private double m_glowIndex = 0;
    private boolean m_isGlowReverse = false;

    private ArrayList<LEDKey> patterns;
    private int patternsIndex;

    /**
     * Speed Values for Moving and Snake colors
     * Time values listed are calculated on 20ms clock
     */
    public enum Speed {
        /**
         * Updates every 1 second or 50 loop iterations.
         */
        Slow,
        /**
         * Updates every 0.5 seconds or 25 loop iterations.
         */
        Fast,
        /**
         * Updates every 0.2 seconds or 10 loop iterations.
         */
        VeryFast,
        /**
         * Updates every 0.1 seconds or 5 loop iterations.
         */
        Ridiculous,
        /**
         * Updates every 0.02 seconds or 1 loop iteration.
         */
        Ludicrous
    }

    /**
     * Preset patterns that can be used in any of the CT_LEDStrip color methods.
     */
    public enum ColorPattern {

        Patriotic(Color.kRed, Color.kWhite, Color.kBlue),
        Christmas(Color.kRed, Color.kGreen),
        Cougartech(Color.kOrange, Color.kBlack),
        /**
         * Just using this pattern in the doMovingColors() method will not do the
         * specific snake movement. Use doSnake() with this pattern for full effect.
         * 
         * Recommended background: Black
         */
        SnakeDefault(Color.kGreen, Color.kRed, Color.kGreen, Color.kRed, Color.kGreen, Color.kRed, Color.kWhite),
        SnakePacman(
                Color.kLightBlue, Color.kLightBlue, Color.kLightBlue, Color.kBlack,
                Color.kPink, Color.kPink, Color.kPink, Color.kBlack,
                Color.kRed, Color.kRed, Color.kRed, Color.kBlack,
                Color.kOrange, Color.kOrange, Color.kOrange, Color.kBlack, Color.kBlack,
                Color.kYellow, Color.kYellow, Color.kYellow);

        private Color[] pattern;

        ColorPattern(Color... pattern) {
            this.pattern = pattern;
        }

        public Color[] getPattern() {
            return pattern;
        }
    }

    protected enum MovementType {
        /**
         * Uses setColor()
         */
        Normal,
        /**
         * Uses doMovingColors()
         */
        Moving,
        /**
         * Uses doSnake()
         */
        Snake,
        /**
         * Uses doRainbow()
         */
        Rainbow,
        /**
         * Uses doGlow()
         */
        Glow
    }

    /**
     * Due to the complexity of altering RGB values to simulate a glow,
     * the following preset colors are simple enough to use for a gradient
     * without some complex formula.
     */
    public enum GlowColor {

        Red(0),
        Green(1),
        Blue(2),
        Yellow(3),
        Purple(4),
        Cyan(5),
        White(6);

        private int colorKey;

        GlowColor(int colorKey) {
            this.colorKey = colorKey;
        }

        public int getColorKey() {
            return colorKey;
        }
    }

    /**
     * Creates a default CT_LED instance where the LED length is the max.
     * 
     * @param PWMPort the PWM port the LED Strip is connected.
     */
    public CT_LEDStrip(int PWMPort) {
        this(PWMPort, 150); // 150 is the max amount of LEDS on a standard LED strip
    }

    /**
     * Creates a CT_LED instance where the LED length can be passed in.
     * 
     * @param PWMPort the PWM port the LED Strip is connected.
     * @param length  the amount of individual LEDS that will be turned on and
     *                affected by color chanes.
     */
    public CT_LEDStrip(int PWMPort, int length) {
        super(PWMPort);

        m_LEDBuffer = new AddressableLEDBuffer(length);
        setLength(m_LEDBuffer.getLength());
        setData(m_LEDBuffer);
        start();

        patterns = new ArrayList<>();
        patternsIndex = 0;
    }

    /**
     * Resets all the moving color variables, allows the changing of speed for
     * example.
     */
    public void reset() {
        m_rainbowFirstPixelHue = 0;

        m_snakeLoopIndex = 0;
        m_snakeCounter = 0;

        m_movingColorsCounter = 0;
        m_movingColorIndex = 0;

        m_glowCounter = 0;
    }

    /**
     * Returns if the counter has reached its specific loop iteration.
     * 
     * @param speed   the speed at which the counter will be compared to.
     * @param counter the value at which the speed will be compared to.
     * @return if the correct amount of loop iterations is completed.
     */
    private boolean hasWaited(Speed speed, int counter) {
        if (counter == 50 && speed == Speed.Slow ||
                counter == 25 && speed == Speed.Fast ||
                counter == 10 && speed == Speed.VeryFast ||
                counter == 5 && speed == Speed.Ridiculous ||
                counter == 1 && speed == Speed.Ludicrous) {

            return true;
        } else {
            return false;
        }
    }

    /**
     * Sets the colors of the LED strip. Colors passed in will retain their order on
     * the LED strip.
     * For example, if (Color.kRed, Color.kBlue) is passed in, the pattern "red,
     * blue, red, blue, red, blue"
     * will repeat on the LED strip.
     * If the LED Strip draws too much current, using the color white too often or
     * having the LED strip length too long
     * might cause colors to be dimmed or show up on the LED strip incorrectly.
     * (e.g. 2 amps will not support a 150 white LED strip)
     * 
     * @param color the colors to be shown on the LED Strip.
     */
    public void setColor(Color... color) {

        if (color.length > 0) {

            int colorIndex = 0;

            for (int ledIndex = 0; ledIndex < m_LEDBuffer.getLength(); ledIndex++) {

                if (colorIndex == color.length - 1) { // Cycles the colorIndex variable to rotate through the color
                                                      // array.
                    colorIndex = 0;
                } else {
                    colorIndex++;
                }

                m_LEDBuffer.setLED(ledIndex, color[colorIndex]);

            }

            setData(m_LEDBuffer);
        }
    }

    /**
     * Sets the colors of the LED strip. Colors passed in will retain their order on
     * the LED strip.
     * For example, if (Color.kRed, Color.kBlue) is passed in, the pattern "red,
     * blue, red, blue, red, blue"
     * will repeat on the LED strip.
     * If the LED Strip draws too much current, using the color white too often or
     * having the LED strip length too long
     * might cause colors to be dimmed or show up on the LED strip incorrectly.
     * (e.g. 2 amps will not support a 150 white LED strip)
     * 
     * @param colorPattern the preset color pattern to be shown on the led strip.
     */
    public void setColor(ColorPattern colorPattern) {
        this.setColor(colorPattern.getPattern());
    }

    /**
     * Sets colors that will move along the LED strip. This method should be called
     * in the periodic of a subsystem to gain full effect.
     * 
     * @param speed the speed at which the snake will move.
     * @param color the colors that will be moving on the LED strip. More than 1
     *              color should be passed in or the method
     *              will not work.
     */
    public void doMovingColors(Speed speed, Color... color) {

        if (color.length <= 1) {
            System.out
                    .println("Too little amount of colors passed in, pass in more colors or use the setColor method.");
            return;
        }

        int colorLoopIndex = m_movingColorIndex;

        // Waits for the correct amount of loop iterations to complete.
        if (hasWaited(speed, m_movingColorsCounter)) {

            // Loops through the whole LED strip.
            for (int ledIndex = 0; ledIndex < m_LEDBuffer.getLength(); ledIndex++) {

                // Increment the color loop index
                colorLoopIndex++;

                // Reset the color loop index back to 0 if it excedes the amount of values in
                // the array.
                if (colorLoopIndex > (color.length - 1)) {
                    colorLoopIndex = 0;
                }

                m_LEDBuffer.setLED(ledIndex, color[colorLoopIndex]);
            }

            // Increment the color index so the color pattern is off by one on the next time
            // this method is run.
            // This gives the effect of the colors moving down the LED strip.
            m_movingColorIndex++;

            // Put the color index back to 0 if it excedes the amount of values in the
            // array.
            if (m_movingColorIndex > (color.length - 1)) {
                m_movingColorIndex = 0;
            }
            m_movingColorsCounter = 0;

        } else {
            m_movingColorsCounter++;
        }

        setData(m_LEDBuffer);
    }

    /**
     * Sets colors that will move along the LED strip. This method should be called
     * in the periodic of a subsystem to gain full effect.
     * 
     * @param speed        the speed at which the snake will move.
     * @param colorPattern the preset color pattern to be shown on the led strip.
     */
    public void doMovingColors(Speed speed, ColorPattern colorPattern) {
        this.doMovingColors(speed, colorPattern.getPattern());
    }

    /**
     * Creates a snake pattern on the LED strip where the pattern contained in the
     * snakeColorPattern array moves over the background color. This method should
     * be called in the periodic of a subsystem to gain full effect.
     * 
     * @param speed             the speed at which the snake will move.
     * @param backgroundColor   the color that will be the background that the snake
     *                          will travel over.
     * @param snakeColorPattern the snake pattern that will traverse the LED strip.
     *                          Length needs to be greater than 0 or the method will
     *                          not work.
     */
    public void doSnake(Speed speed, Color backgroundColor, Color[] snakeColorPattern) {

        if (snakeColorPattern.length == 0) {
            System.out.println(
                    "Snake length is zero, create a longer snake by making the snakeColorPattern array longer.");
            return;
        }

        // Waits for the correct amount of loop iterations to complete.
        if (hasWaited(speed, m_snakeCounter)) {

            // Loops through the whole LED strip.
            for (int ledIndex = 0; ledIndex < m_LEDBuffer.getLength(); ledIndex++) {

                // Checks if the current led is the one that will start the snake.
                if (ledIndex == m_snakeLoopIndex) {

                    int endOfSnakeIndex = ledIndex + snakeColorPattern.length;
                    int currentColorIndex = 0;

                    // Loops through the entire snake.
                    for (int snakeIndex = ledIndex; snakeIndex < m_LEDBuffer.getLength()
                            + snakeColorPattern.length; snakeIndex++) {

                        // If the current color index is outside the color array, that means the snake
                        // is complete.
                        if (currentColorIndex == snakeColorPattern.length) {
                            break;
                        }

                        int newSnakeIndex = snakeIndex;

                        // Moves the new snake index back to the beginning when the snake goes over
                        // the end so it flows smoothly to the beginning again.
                        if (snakeIndex > m_LEDBuffer.getLength() - 1) {
                            newSnakeIndex -= m_LEDBuffer.getLength();
                        }

                        m_LEDBuffer.setLED(newSnakeIndex, snakeColorPattern[currentColorIndex]);
                        currentColorIndex++;

                    }

                    // Advances the ledIndex to the end of the snake.
                    ledIndex = endOfSnakeIndex - 1;

                } else {
                    m_LEDBuffer.setLED(ledIndex, backgroundColor);
                }

            }
            m_snakeCounter = 0;

            m_snakeLoopIndex++;

            // Reset the snake loop index back to the beginning when it reaches the end.
            if (m_snakeLoopIndex > m_LEDBuffer.getLength() - 1) {
                m_snakeLoopIndex = 0;
            }

        } else {
            m_snakeCounter++;
        }

        setData(m_LEDBuffer);
    }

    /**
     * Creates a snake pattern on the LED strip where the pattern contained in the
     * snakeColorPattern array moves over the background color. This method should
     * be called in the periodic of a subsystem to gain full effect.
     * 
     * @param speed           the speed at which the snake will move.
     * @param backgroundColor the color that will be the background that the snake
     *                        will travel over.
     * @param colorPattern    the preset color pattern to be shown on the led strip.
     */
    public void doSnake(Speed speed, Color backgroundColor, ColorPattern pattern) {
        this.doSnake(speed, backgroundColor, pattern.getPattern());
    }

    /**
     * Does a glow appearance across the LED strip. This method should be
     * called in the periodic of a subsytem to gain full effect.
     * 
     * @param glowColor preset color
     */
    public void doGlow(GlowColor glowColor) {

        if (hasWaited(Speed.Ludicrous, m_glowCounter)) {

            setGlowColor(glowColor.getColorKey());

            if (!m_isGlowReverse) {
                if (m_glowIndex < GLOW_MAX) {
                    m_glowIndex += INCREASE_VALUE;
                } else {
                    m_isGlowReverse = true;
                }
            } else {
                if (m_glowIndex > GLOW_MIN) {
                    m_glowIndex -= INCREASE_VALUE;
                } else {
                    m_isGlowReverse = false;
                }
            }

            m_glowCounter = 0;
        } else {
            m_glowCounter++;
        }
    }

    /**
     * Sets the color with the glow index in the correct space to alter the color's
     * shade.
     */
    private void setGlowColor(int colorKey) {
        switch (colorKey) {
            case 0:
                setColor(new Color(m_glowIndex, 0, 0));
                break;
            case 1:
                setColor(new Color(0, m_glowIndex, 0));
                break;
            case 2:
                setColor(new Color(0, 0, m_glowIndex));
                break;
            case 3:
                setColor(new Color(m_glowIndex, m_glowIndex, 0));
                break;
            case 4:
                setColor(new Color(m_glowIndex, 0, m_glowIndex));
                break;
            case 5:
                setColor(new Color(0, m_glowIndex, m_glowIndex));
                break;
            case 6:
                setColor(new Color(m_glowIndex, m_glowIndex, m_glowIndex));
                break;
            default:
                System.out.println("Error in CT_LEDStrip, unexpected glow color key found: " + colorKey);
        }
    }

    /**
     * Creates a rainbow effect on the LED strip.
     * This method should be called in the periodic of a subsystem to gain full
     * effect.
     * 
     * Code is from
     * https://docs.wpilib.org/en/stable/docs/software/actuators/addressable-leds.html
     * with a few modifcations to make it more readable and standard.
     */
    public void doRainbow() {
        // For every pixel
        for (int i = 0; i < m_LEDBuffer.getLength(); i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            int hue = (m_rainbowFirstPixelHue + (i * 180 / m_LEDBuffer.getLength())) % 180;
            // Set the value
            m_LEDBuffer.setHSV(i, hue, 255, 128);
        }

        // Increase by to make the rainbow "move"
        m_rainbowFirstPixelHue += 3;
        // Check bounds
        m_rainbowFirstPixelHue %= 180;

        setData(m_LEDBuffer);
    }

    /**
     * Methods for adding patterns
     */

    public void addNormalPattern(String key, Color... colorPattern) {
        patterns.add(new LEDKey(key, null, null, MovementType.Normal, null, colorPattern));
    }

    public void addMovingPattern(String key, Speed speed, Color... colorPattern) {
        patterns.add(new LEDKey(key, speed, null, MovementType.Moving, null, colorPattern));
    }

    public void addSnakePattern(String key, Speed speed, Color backgroundColor, Color... colorPattern) {
        patterns.add(new LEDKey(key, speed, backgroundColor, MovementType.Snake, null, colorPattern));
    }

    public void addRainbowPattern() {
        patterns.add(new LEDKey("Rainbow", null, null, MovementType.Rainbow, null));
    }

    public void addGlowPattern(String key, GlowColor glowColor) {
        patterns.add(new LEDKey(key, null, null, MovementType.Glow, glowColor));
    }

    /**
     * Indexes the pattern list up or down
     * 
     * @param goRight if true, the index will be increased by 1. If false the index
     *                will be decreased by 1
     */
    public void indexPattern(boolean goRight) {
        reset();
        if (goRight) {
            if (patternsIndex == patterns.size() - 1) {
                patternsIndex = 0;
            } else {
                patternsIndex++;
            }
        } else {
            if (patternsIndex == 0) {
                patternsIndex = patterns.size() - 1;
            } else {
                patternsIndex--;
            }
        }
    }

    /**
     * Gets the current color pattern. This method should be called in a periodic to
     * gain full effect.
     * 
     * @return the color method that will change the leds.
     */
    public Runnable getCurrentPattern() {
        LEDKey curKey = patterns.get(patternsIndex);
        MovementType movementType = curKey.movementType;
        Speed speed = curKey.speed;
        Color[] colorPattern = curKey.colorPattern;

        if (movementType == MovementType.Normal) {
            return () -> setColor(colorPattern);
        } else if (movementType == MovementType.Moving) {
            return () -> doMovingColors(speed, colorPattern);
        } else if (movementType == MovementType.Snake) {
            return () -> doSnake(speed, curKey.backgroundColor, colorPattern);
        } else if (movementType == MovementType.Glow) {
            return () -> doGlow(curKey.glowColor);
        } else {
            return () -> doRainbow();
        }

    }

    /**
     * Gets the key of the current pattern
     * 
     * @return the key first entered when the pattern was entered
     */
    public String getCurrentPatternString() {
        return patterns.get(patternsIndex).key;
    }
}

/**
 * The class that holds the key information for specific patterns
 */
class LEDKey {

    public Color[] colorPattern;
    public Speed speed;
    public Color backgroundColor;
    public MovementType movementType;
    public GlowColor glowColor;
    public String key;

    LEDKey(String key, Speed speed, Color backgroundColor, MovementType movementType, GlowColor glowColor,
            Color... colorPattern) {
        this.key = key;
        this.glowColor = glowColor;
        this.colorPattern = colorPattern;
        this.speed = speed;
        this.backgroundColor = backgroundColor;
        this.movementType = movementType;
    }
}