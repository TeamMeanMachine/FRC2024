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
import kotlin.math.sin

object LedControl : Subsystem ("LedControl"){

    private val table = NetworkTableInstance.getDefault().getTable("Led Control")
    val patternEntry = table.getEntry("Pattern")

    val led = AddressableLED(Leds.LED_PORT)
    val ledBuffer = AddressableLEDBuffer(Leds.LED_LENGTH)
    val controllerTestLength = 12
    val controlerTestEnabled = false
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

        staticRainbow(255, 2)
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

                    LedPatterns.INTAKE -> updateBlink(Color.kRed, 0.5)
                    LedPatterns.SHOOTING -> updateBlink(Color.kRed, 0.5)
                    LedPatterns.HOLDING -> updateBlink(Color.kRed, 0.5)
                    LedPatterns.AUTO -> updateBlink(Color.kRed, 0.5)
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

//    fun startRainbow(brightness: Int = 255, rainbowCount: Int = 1, speed: Int = 1) {
//        stopAnimations()
//        animationData = arrayOf(brightness, rainbowCount, speed)
//        rainbowOn = true
//    }

//    fun startBlinking(color: Color, delay: Int, brightness: Int = 255) {
//        stopAnimations()
//        animationColor = color
//        animationData = arrayOf(delay, brightness)
//        blinkingOn = true
//    }

    fun staticRainbow(brightness: Int = 255, rainbowCount: Int = 1) {
        //stopAnimations()
        for (i in 0 until ledBuffer.length) {
            ledBuffer.setHSV(i, linearMap(0.0, ledBuffer.length.toDouble() / rainbowCount, 0.0, 180.0, rainbowFirstPixelHue + i.toDouble()).toInt(), 255, brightness)
//            ledBuffer.setRGB(i, 255, 0, 0)
        }
    }

    private fun updateBlink(color: Color, delay: Double, brightness: Int = 255) {
        if (timer.get() > delay) {
            for (i in 0 until ledBuffer.length) {
                ledBuffer.setRGB(i, (color.red * 255).toInt(), (color.green * 255).toInt(), (color.blue * 255).toInt())
            }
        } else {
            for (i in 0 until ledBuffer.length) {
                ledBuffer.setRGB(i, 0, 0, 0)
            }
        }
        if (timer.get() > delay * 2) timer.start()
    }

    private fun updateRainbow(brightness: Int = 255, rainbowCount: Int = 1, speed: Int = 1) {
        for (i in 0 until ledBuffer.length) {
            ledBuffer.setHSV(i, linearMap(0.0, ledBuffer.length.toDouble() / rainbowCount, 0.0, 180.0, rainbowFirstPixelHue + i.toDouble()).toInt(), 255, brightness)
        }
        rainbowFirstPixelHue += speed
        rainbowFirstPixelHue %= 180

    }

    fun updatePulse(color: Color, rate: Double) {
        //stopAnimations()

        val brightness = ((sin(timer.get() * rate * 2 * PI) + 1.0) / 2.0) * 255

        for (i in 0 until ledBuffer.length) {
            ledBuffer.setRGB(i, (color.red * brightness).toInt(), (color.green * brightness).toInt(), (color.blue * brightness).toInt())
        }
    }

    fun setSolid(color: Color, brightness: Int = 255) {
        //stopAnimations()
        for (i in 0 until ledBuffer.length) {
            //println(color.red)

            ledBuffer.setRGB(i, (color.red * brightness).toInt(), (color.green * brightness).toInt(), (color.blue * brightness).toInt())
        }
    }

    fun setDotted(color: Color, spacing: Int) {
        //stopAnimations()

        for (i in 0 until ledBuffer.length) {
            if (i % spacing == 1) {
                ledBuffer.setRGB(i, (color.red * 255).toInt(), (color.green * 255).toInt(), (color.blue * 255).toInt())
                //println("dot")
            }
            else {
                ledBuffer.setRGB(i, 0, 0, 0)
                //println("no dot")
            }
        }
    }

    fun overlayDotted(color: Color, spacing: Int) {
        //stopAnimations()
        for (i in 0 until ledBuffer.length) {
            if (i % spacing == 1) {
                ledBuffer.setRGB(i, (color.red * 255).toInt(), (color.green * 255).toInt(), (color.blue * 255).toInt())
            }
        }
    }

    fun setFractionColorFromMid(color: Color, value: Double, max: Double, min: Double = 0.0) {
        //stopAnimations()

        try {
            var newValue: Double

            if (min > max) throw IllegalArgumentException()

            if (value > max) newValue = max
            else if (value < min) newValue = min
            else newValue = value - min

            val halfLength = ledBuffer.length / 2
            val newMax = max - min
            val numOfLedsOn = ((newValue / newMax) * halfLength).toInt()
            for (i in 0 until numOfLedsOn) {
                ledBuffer.setRGB(
                    halfLength + i,
                    (color.red * 255).toInt(),
                    (color.green * 255).toInt(),
                    (color.blue * 255).toInt()
                )
                ledBuffer.setRGB(
                    halfLength - i,
                    (color.red * 255).toInt(),
                    (color.green * 255).toInt(),
                    (color.blue * 255).toInt()
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