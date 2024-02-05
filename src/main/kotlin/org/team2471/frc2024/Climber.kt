package org.team2471.frc2024

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DutyCycleEncoder
import edu.wpi.first.wpilibj.Relay
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.actuators.MotorController
import org.team2471.frc.lib.actuators.SparkMaxID
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem

object Climber: Subsystem("Climber") {
    private val table = NetworkTableInstance.getDefault().getTable("")

    private val climberPercentEntry = table.getEntry("Climber Percent")
    private val climberCurrentEntry = table.getEntry("Climber Current")
    private val climberEncoderEntry = table.getEntry("Climber Encoder Value")

    private val climberMotor = MotorController(SparkMaxID(Sparks.CLIMBER))

    private val relay = Relay(Solenoids.CLIMB_SWITCH)

    private var relayOn: Boolean = false

    fun relayOn() {
        println("******************************************RELAY ON")
//        relay.set(Relay.Value.kForward)
        relayOn = true
    }

    fun relayOff() {
        println("*************************************RELAY OFF")
//        relay.set(Relay.Value.kReverse)
        relayOn = false
    }

    init {
        GlobalScope.launch {
            periodic {
                climberEncoderEntry.setDouble(climberMotor.position)

                if (relayOn) {
                    relay.set(Relay.Value.kForward)
                } else {
                    relay.set(Relay.Value.kOff)

                }
            }
        }
    }
}