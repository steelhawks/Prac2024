package frc.robot.subsystems

import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.*
import frc.robot.Constants


object LEDSubsystem : SubsystemBase() {
    private var LEDStrip = AddressableLED(Constants.LED.PORT)
    private var LEDBuffer = AddressableLEDBuffer(Constants.LED.LENGTH)

    private var lastChange = 0.0
    private var isOn = false
    private var waveIndex = 0
    private var rainbowStart = 0
    private var bounceWaveIndex = 0

    private var bounceWaveDirection = BounceWaveDirection.FORWARD
    private var _currentColor = LEDColor.OFF

    private const val WAVE_LENGTH = 6
    private const val BOUNCE_WAVE_LENGTH = 6

    private var fadeMultiplier = 0.0
    private var fadeDirection = FadeDirection.IN

    private var stripStart: Int? = null
    private var stripLength: Int? = null

    enum class LEDColor(
        var r: Int, var g: Int, var b: Int
    ) {
        PURPLE(70, 2, 115),
        YELLOW(150, 131, 2),
        RED(255, 0, 0),
        BLUE(0, 0, 255),
        GREEN(0, 255, 0),
        WHITE(255, 255, 255),
        CYAN(0, 255, 255),
        ORANGE(252, 144, 3),
        MAGENTA(255, 0, 255),
        PINK(255, 20, 147),
        OFF(0, 0, 0);
    }

    enum class LEDMode {
        STATIC,
        WAVE,
        RAINBOW,
        PULSE,
    }

    enum class FadeDirection {
        IN, OUT
    }

    enum class BounceWaveDirection {
        FORWARD,
        BACKWARD
    }

    init {
        stripStart = Constants.LED.LENGTH / 2
        stripLength = Constants.LED.LENGTH / 2

        LEDStrip.setLength(LEDBuffer.length)
        LEDStrip.setData(LEDBuffer)
        LEDStrip.start()
    }

    fun setColor(color: LEDColor) {
        for (i in 0 until LEDBuffer.length) {
            LEDBuffer.setRGB(i, color.r, color.g, color.b)
        }

        _currentColor = color;
        LEDStrip.setData(LEDBuffer)
    }

    fun pulse(color: LEDColor, interval: Double) {
        val timestamp = Timer.getFPGATimestamp()

        if (timestamp - lastChange!! > interval) {
            lastChange = timestamp
            isOn = !isOn!!
        }

        if (isOn) {
            setColor(color)
        } else {
            stop()
        }
    }

    fun wave(color: LEDColor) {
        for (i in 0 until stripLength!!) {
            if ((i >= waveIndex && i < waveIndex + WAVE_LENGTH)
                || (waveIndex + WAVE_LENGTH > stripLength!! && i < (waveIndex + WAVE_LENGTH) % stripLength!!)
            ) {
                LEDBuffer.setRGB(i, color.r, color.g, color.b)
                LEDBuffer.setRGB(i + stripStart!!, color.r, color.g, color.b)
            } else {
                LEDBuffer.setRGB(i, 0, 0, 0)
                LEDBuffer.setRGB(i + stripStart!!, 0, 0, 0)
            }
        }

        waveIndex++
        waveIndex = (waveIndex % stripLength!!).toInt()

        _currentColor = LEDColor.OFF
        LEDStrip.setData(this.LEDBuffer)
    }

    fun bounceWave(color: LEDColor) {
        for (i in 0 until stripLength!!) {
            if (i >= bounceWaveIndex && i < bounceWaveIndex + BOUNCE_WAVE_LENGTH) {
                LEDBuffer.setRGB(i, color.r, color.g, color.b)
                LEDBuffer.setRGB(i + stripStart!!, color.r, color.g, color.b)
            } else {
                LEDBuffer.setRGB(i, 0, 0, 0)
                LEDBuffer.setRGB(i + stripStart!!, 0, 0, 0)
            }
        }

        if (bounceWaveIndex == 0) {
            bounceWaveDirection = BounceWaveDirection.FORWARD
        } else if (bounceWaveIndex == LEDBuffer.length - BOUNCE_WAVE_LENGTH) {
            bounceWaveDirection = BounceWaveDirection.BACKWARD
        }

        if (bounceWaveDirection == BounceWaveDirection.FORWARD) {
            bounceWaveIndex++
        } else {
            bounceWaveIndex--
        }

        _currentColor = LEDColor.OFF
        LEDStrip.setData(this.LEDBuffer)
    }

    fun rainbow() {
        var i = 0
        while (i < stripLength!!) {
            i %= stripLength!!

            val hue = (rainbowStart + (i * 180 / stripLength!!)) % 180
            LEDBuffer.setHSV(i, hue, 255, 128) // Strip 1
            LEDBuffer.setHSV(i + stripStart!!, hue, 255, 128) // Strip 2
            i++
        }

        _currentColor = LEDColor.OFF
        LEDStrip.setData(LEDBuffer)

        rainbowStart += 1
        rainbowStart %= 180
    }

    fun fade(color: LEDColor) {
        for (i in 0 until LEDBuffer.length) {
            LEDBuffer.setRGB(
                i,
                (color.r * fadeMultiplier).toInt(),
                (color.g * fadeMultiplier).toInt(),
                (color.b * fadeMultiplier).toInt()
            )
        }

        if (fadeMultiplier <= 0.02) {
            fadeDirection = FadeDirection.IN
        } else if (fadeMultiplier >= 0.98) {
            fadeDirection = FadeDirection.OUT
        }

        if (fadeDirection == FadeDirection.IN) {
            fadeMultiplier += 0.02
        } else if (fadeDirection == FadeDirection.OUT) {
            fadeMultiplier -= 0.02
        }

        _currentColor = color
        LEDStrip.setData(LEDBuffer)
    }

    fun flashCommand(color: LEDColor, interval: Double, time: Double): Command {
        return ParallelDeadlineGroup(
            WaitCommand(time),
            Commands.run({
                this.pulse(color, interval)
            }, this)
        )
    }

    fun fadeCommand(color: LEDColor, interval: Double, time: Double): Command {
        return ParallelDeadlineGroup(
            Commands.run({
                this.fade(color)
            }, this)
        )
    }


    private val currentColor: LEDColor
        get() = _currentColor

    fun stop() {
        setColor(LEDColor.OFF)
    }
}
