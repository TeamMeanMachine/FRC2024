package org.team2471.frc2024

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj.util.Color
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.math.linearMap
import org.team2471.frc.lib.util.Timer
import kotlin.math.PI
import kotlin.math.absoluteValue
import kotlin.math.sin

object LedControl : Subsystem ("LedControl"){

    private val table = NetworkTableInstance.getDefault().getTable("Led Control")
    val patternEntry = table.getEntry("Pattern")

    private val timer = Timer()

    val controlerTestEnabled = false

    val led = AddressableLED(Leds.LED_PORT)
    val ledBuffer = AddressableLEDBuffer(Leds.LED_LENGTH)
    val maxBrightness = 255
    var rainbowFirstPixelHue = 0
    var pattern = LedPatterns.INIT


    private var meanOrange = Color(255, 25, 0)

    init {
        println("LEDS YAYYYYY")
        led.setLength(ledBuffer.length)

        staticRainbow(maxBrightness, 2)
        led.setData(ledBuffer)

        led.start()
        timer.start()


        GlobalScope.launch {
            periodic {

                patternEntry.setString(pattern.name)

                when (pattern) {
                    LedPatterns.INIT -> updateBlink(meanOrange, 0.2)
                    LedPatterns.ENABLED -> updatePulse(Color.kRed, 1.0)
                    LedPatterns.DISABLED -> setSolid(Color.kRed, 100)

                    LedPatterns.INTAKING -> updateBlink(Color.kYellow, 0.2)
                    LedPatterns.SHOOTING -> setSolid(Color.kWhite)
                    LedPatterns.HOLDING -> setSolid(Color.kGreen)

                    LedPatterns.RAMPING -> setFractionColorFromMid(Color.kGreen, Shooter.averageRpm, Shooter.averageRpmSetpoint)
                    LedPatterns.READY -> updateBlink(Color.kGreen, 0.06)
                }

                if (controlerTestEnabled and !isEnabled) {
                    if (OI.driverController.b or (OI.driverController.leftThumbstick.length >= 1)) {
                        overlayDotted(Color(0.5, 0.5, 0.5), 2)
                    } else {
                        overlayDotted(Color.kBlack, 2)
                    }
                }
                led.setData(ledBuffer)
            }
        }
    }

    override suspend fun default() {
        periodic {
            pattern = if (Robot.isTeleopEnabled && Intake.intakeState != Intake.IntakeState.SHOOTING && (Shooter.motorRpmTop - Shooter.rpmTopSetpoint).absoluteValue + (Shooter.motorRpmBottom - Shooter.rpmBottomSetpoint).absoluteValue < 500.0 && Shooter.rpmTopSetpoint + Shooter.rpmBottomSetpoint > 20.0) {
                LedPatterns.READY
            } else if (Shooter.averageRpmSetpoint > 10.0 && Intake.intakeState != Intake.IntakeState.SHOOTING) {
                LedPatterns.RAMPING
            } else {
                when (Intake.intakeState) {
                    Intake.IntakeState.INTAKING -> LedPatterns.INTAKING
                    Intake.IntakeState.HOLDING -> LedPatterns.HOLDING
                    Intake.IntakeState.SHOOTING -> LedPatterns.SHOOTING
                    else -> LedPatterns.ENABLED
                }
            }
        }
    }

    override fun onDisable() {
        pattern = LedPatterns.DISABLED
    }


    /**
     * staticRainbow:
     * Sets the entire LED strip to a static rainbow pattern
     * @param brightness: from 0 (off) to maxBrightness for led brightness
     * @param rainbowCount: number of full rainbows on the strip
     */
    fun staticRainbow(brightness: Int = maxBrightness, rainbowCount: Int = 1) {
        for (i in 0 until ledBuffer.length) {
            ledBuffer.setHSV(i, linearMap(0.0, ledBuffer.length.toDouble() / rainbowCount, 0.0, 180.0, rainbowFirstPixelHue + i.toDouble()).toInt(), 255, brightness)
        }
    }


    /**
     * updateBlink:
     * Toggles the entire LED strip on/off every delay seconds
     * @param color: wpilib color object for led color
     * @param delay: time in seconds in between toggling on/off
     * @param brightness: from 0 (off) to maxBrightness for led brightness
     */
    private fun updateBlink(color: Color, delay: Double, brightness: Int = maxBrightness) {
        if (timer.get() > delay) {
            for (i in 0 until ledBuffer.length) {
                ledBuffer.setRGB(i, (color.red * brightness).toInt(), (color.green * brightness).toInt(), (color.blue * brightness).toInt())
            }
        } else {
            for (i in 0 until ledBuffer.length) {
                ledBuffer.setRGB(i, 0, 0, 0)
            }
        }
        if (timer.get() > delay * 2) timer.start()
    }

    /**
     * updateRainbow:
     * Sets the entire LED strip to a rainbow pattern and increments the hue by colorIncrement
     * @param brightness: from 0 (off) to maxBrightness for led brightness
     * @param rainbowCount: number of full rainbows on the strip
     * @param colorIncrement: how much the hue is incremented every time the function is run
     */
    private fun updateRainbow(brightness: Int = maxBrightness, rainbowCount: Int = 1, colorIncrement: Int = 1) {
        staticRainbow(brightness, rainbowCount)
        rainbowFirstPixelHue += colorIncrement
        rainbowFirstPixelHue %= 180

    }

    /**
     * updatePulse:
     * Sets the entire LED strip to a color with brightness multiplied by a raised sine pulse
     * @param color: wpilib color object for led color
     * @param period: pulse period in seconds
     */
    fun updatePulse(color: Color, period: Double) {

        val brightness = ((sin(timer.get() * period * 2 * PI) + 1.0) / 2.0) * maxBrightness

        for (i in 0 until ledBuffer.length) {
            ledBuffer.setRGB(i, (color.red * brightness).toInt(), (color.green * brightness).toInt(), (color.blue * brightness).toInt())
        }
    }

    /**
     * setSolid:
     * Sets the entire LED strip to a solid color
     * @param color: wpilib color object for LED color
     * @param brightness: from 0 (off) to maxBrightness for led brightness
     */
    fun setSolid(color: Color, brightness: Int = maxBrightness) {
        for (i in 0 until ledBuffer.length) {
            ledBuffer.setRGB(i, (color.red * brightness).toInt(), (color.green * brightness).toInt(), (color.blue * brightness).toInt())
        }
    }

    /**
     * setDotted:
     * Sets every spacing + 1th LED to a color and the rest to off
     * @param color: wpilib color object for led color
     * @param spacing: number of off LEDs in between on ones
     * @param brightness: from 0 (off) to maxBrightness for led brightness
     */
    fun setDotted(color: Color, spacing: Int, brightness: Int = maxBrightness) {

        for (i in 0 until ledBuffer.length) {
            if (i % (spacing + 1) == 0) {
                ledBuffer.setRGB(i, (color.red * brightness).toInt(), (color.green * brightness).toInt(), (color.blue * brightness).toInt())
                //println("dot")
            }
            else {
                ledBuffer.setRGB(i, 0, 0, 0)
                //println("no dot")
            }
        }
    }

    /**
     * overlayDotted:
     * setDotted but without setting to off
     */
    fun overlayDotted(color: Color, spacing: Int, brightness: Int = maxBrightness) {
        for (i in 0 until ledBuffer.length) {
            if (i % (spacing + 1) == 1) {
                ledBuffer.setRGB(i, (color.red * brightness).toInt(), (color.green * brightness).toInt(), (color.blue * brightness).toInt())
            }
        }
    }

    /**
     * setFractionColorFromMid:
     * Creates an LED bar graph centered on the middle of the strip where the width is proportional to value
     * @param color: wpilib color object for LED color
     * @param value: number to be graphed
     * @param min: lower bound of value
     * @param max: upper bound of value
     */
    fun setFractionColorFromMid(color: Color, value: Double, max: Double, min: Double = 0.0) {

        try {
            for (i in 0 until ledBuffer.length) {
                ledBuffer.setRGB(i, 0, 0, 0)
            }

            var newValue: Double

            if (min > max) throw IllegalArgumentException()

            if (value > max) newValue = max - min
            else if (value < min) newValue = 0.0
            else newValue = value - min

            val halfLength = (ledBuffer.length / 2.0).toInt()
            val newMax = max - min
            val numOfLedsOn = ((newValue / newMax) * halfLength).toInt()
            for (i in 0 until numOfLedsOn) {
                ledBuffer.setRGB(
                    halfLength + i,
                    (color.red * maxBrightness).toInt(),
                    (color.green * maxBrightness).toInt(),
                    (color.blue * maxBrightness).toInt()
                )
                ledBuffer.setRGB(
                    halfLength - i,
                    (color.red * maxBrightness).toInt(),
                    (color.green * maxBrightness).toInt(),
                    (color.blue * maxBrightness).toInt()
                )
            }
        } catch (e: Exception) {
            println(e)
        }
    }

}

enum class LedPatterns {
    DISABLED,
    ENABLED,
    INIT,
    INTAKING,
    HOLDING,
    SHOOTING,
    RAMPING,
    READY
}