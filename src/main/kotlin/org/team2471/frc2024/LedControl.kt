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
import kotlin.math.floor
import kotlin.math.sin

object LedControl : Subsystem ("LedControl"){

    private val table = NetworkTableInstance.getDefault().getTable("Led Control")
    val patternEntry = table.getEntry("Pattern")

    val led = AddressableLED(Leds.LED_PORT)
    val ledBuffer = AddressableLEDBuffer(Leds.LED_LENGTH)
    val controllerTestLength = 12
    val controlerTestEnabled = false
    val maxBrightness = 255
    //val defaultDisabledColor = Color(255, 25, 0)
    var rainbowFirstPixelHue = 0
//    var blinkCounter = 0
//    var rainbowOn = false
//    var blinkingOn = false
//    var animationData = emptyArray<Int>()
//    var animationColor = Color.kRed
    var pattern = LedPatterns.INIT

    private val timer = Timer()

    init {
        println("LEDS YAYYYYY")
        led.setLength(ledBuffer.length)

        staticRainbow(maxBrightness, 2)
        //setDotted(Color.kRed, 5)
        led.setData(ledBuffer)

        led.start()
        timer.start()

        //startBlinking(Color.kRed, 5)

        GlobalScope.launch {
            periodic(0.02) {
                //updateRainbow(255, 2)
                //when {
                    //rainbowOn -> updateRainbow(animationData[0], animationData[1], animationData[2])
                    //blinkingOn -> updateBlink(animationColor, animationData[0], animationData[1])
                //}

                patternEntry.setString(pattern.name)

                when (pattern) {
                    LedPatterns.INIT -> updateBlink(Color(255, 25, 0), 0.2)
                    LedPatterns.DISABLED -> setSolid(Color.kRed)
                    LedPatterns.ENABLED -> updatePulse(Color.kRed, 1.0)

                    LedPatterns.INTAKE -> updateBlink(Color.kYellow, 0.2)
                    LedPatterns.SHOOTING -> updateBlink(Color.kYellow, 0.04)
                    LedPatterns.HOLDING -> updateBlink(Color.kRed, 0.2)
                    LedPatterns.AUTO -> updatePulse(Color(255, 25, 0), 0.5)
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
            if (pattern != LedPatterns.INIT) {
                if (Robot.isEnabled) {
                    pattern = LedPatterns.ENABLED
                } else {
                    pattern = LedPatterns.DISABLED
                }
            }
        }
    }


//    fun stopAnimations() {
//        rainbowOn = false
//        blinkingOn = false
//    }

//    fun startRainbow(brightness: Int = maxBrightness, rainbowCount: Int = 1, speed: Int = 1) {
//        stopAnimations()
//        animationData = arrayOf(brightness, rainbowCount, speed)
//        rainbowOn = true
//    }

//    fun startBlinking(color: Color, delay: Int, brightness: Int = maxBrightness) {
//        stopAnimations()
//        animationColor = color
//        animationData = arrayOf(delay, brightness)
//        blinkingOn = true
//    }


    /**
     * staticRainbow:
     * Sets the entire LED strip to a static rainbow pattern
     * brightness: from 0 (off) to maxBrightness for led brightness
     * rainbowCount: number of full rainbows on the strip
    */
    fun staticRainbow(brightness: Int = maxBrightness, rainbowCount: Int = 1) {
        //stopAnimations()
        for (i in 0 until ledBuffer.length) {
            ledBuffer.setHSV(i, linearMap(0.0, ledBuffer.length.toDouble() / rainbowCount, 0.0, 180.0, rainbowFirstPixelHue + i.toDouble()).toInt(), 255, brightness)
//            ledBuffer.setRGB(i, 255, 0, 0)
        }
    }


    /**
     * updateBlink:
     * Toggles the entire LED strip on/off every delay seconds
     * color: wpilib color object for led color
     * delay: time in seconds in between toggling on/off
     * brightness: from 0 (off) to maxBrightness for led brightness
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
     * brightness: from 0 (off) to maxBrightness for led brightness
     * rainbowCount: number of full rainbows on the strip
     * colorIncrement: how much the hue is incremented every time the function is run
    */
    private fun updateRainbow(brightness: Int = maxBrightness, rainbowCount: Int = 1, colorIncrement: Int = 1) {
        staticRainbow(brightness, rainbowCount)
        rainbowFirstPixelHue += colorIncrement
        rainbowFirstPixelHue %= 180

    }

    /**
     * updatePulse:
     * Sets the entire LED strip to a color with brightness multiplied by a raised sine pulse
     * color: wpilib color object for led color
     * period: pulse period in seconds
    */
    fun updatePulse(color: Color, period: Double) {
        //stopAnimations()

        val brightness = ((sin(timer.get() * period * 2 * PI) + 1.0) / 2.0) * maxBrightness

        for (i in 0 until ledBuffer.length) {
            ledBuffer.setRGB(i, (color.red * brightness).toInt(), (color.green * brightness).toInt(), (color.blue * brightness).toInt())
        }
    }

    /**
     * setSolid:
     * Sets the entire LED strip to a solid color
     * color: wpilib color object for LED color
     * brightness: from 0 (off) to maxBrightness for led brightness
     */
    fun setSolid(color: Color, brightness: Int = maxBrightness) {
        //stopAnimations()
        for (i in 0 until ledBuffer.length) {
            //println(color.red)

            ledBuffer.setRGB(i, (color.red * brightness).toInt(), (color.green * brightness).toInt(), (color.blue * brightness).toInt())
        }
    }

    /**
     * setDotted:
     * Sets every spacing + 1th LED to a color and the rest to off
     * color: wpilib color object for led color
     * spacing: number of off LEDs in between on ones
     * brightness: from 0 (off) to maxBrightness for led brightness
     */
    fun setDotted(color: Color, spacing: Int, brightness: Int = maxBrightness) {
        //stopAnimations()

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
        //stopAnimations()
        for (i in 0 until ledBuffer.length) {
            if (i % (spacing + 1) == 1) {
                ledBuffer.setRGB(i, (color.red * brightness).toInt(), (color.green * brightness).toInt(), (color.blue * brightness).toInt())
            }
        }
    }

    /**
     * setFractionColorFromMid:
     * Creates an LED bar graph centered on the middle of the strip where the width is proportional to value
     * color: wpilib color object for LED color
     * value: number to be graphed
     * min: lower bound of value
     * max: upper bound of value
     */
    fun setFractionColorFromMid(color: Color, value: Double, max: Double, min: Double = 0.0) {
        //stopAnimations()

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
    INTAKE,
    HOLDING,
    SHOOTING,
    AUTO
}