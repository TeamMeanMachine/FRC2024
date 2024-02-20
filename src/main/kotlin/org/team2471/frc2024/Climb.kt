package org.team2471.frc2024

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DutyCycleEncoder
import edu.wpi.first.wpilibj.Relay
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.actuators.MotorController
import org.team2471.frc.lib.actuators.SparkMaxID
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.units.Length
import org.team2471.frc.lib.units.inches

object Climb: Subsystem("Climb") {
    private val table = NetworkTableInstance.getDefault().getTable("Climb")

    private val climberPercentEntry = table.getEntry("Climb Percent")
    private val climberCurrentEntry = table.getEntry("Climb Current")
    private val climberEncoderEntry = table.getEntry("Climb Encoder Value")
    private val climberHeightEntry = table.getEntry("Motor Height")
    private val climbSetpointEntry = table.getEntry("Climb Setpoint")
    private val relayOnEntry = table.getEntry("Relay On")

    val climberMotor = MotorController(SparkMaxID(Sparks.CLIMBER))
    val climberEncoder = DutyCycleEncoder(DigitalSensors.CLIMBER)
    private val relay = Relay(Solenoids.CLIMB_SWITCH)

    val climberHeight: Length
        get() = climberMotor.position.inches
    var climbSetpoint: Length = climberHeight
        set(value) {
            var safeValue = value.asInches.coerceIn(MIN_CLIMB_INCHES, MAX_CLIMB_INCHES).inches
            if (relayOn && safeValue > climberHeight) {
                safeValue = climberHeight
            }
//            println("going to value $safeValue")
//            relayOn = (safeValue > climberHeight) //if going up, turn relay on. if going down, turn relay off.
            climberMotor.setPositionSetpoint(safeValue.asInches)
            field = safeValue
        }


    private var relayOn: Boolean
        get() = relay.get() == Relay.Value.kForward
        set(value) {
            if (value) {
                relay.set(Relay.Value.kForward)
            } else {
                relay.set(Relay.Value.kOff)
            }
        }

    const val MIN_CLIMB_INCHES = 23.5
    const val MAX_CLIMB_INCHES = 45.625

    init {
        climberMotor.config {
            feedbackCoefficient = 0.4331 * 1.1538//1.0/*23.68 * 0.0985 / 5.337*/ //need ticks (or rotations) per inch and maybe gear ratio
            currentLimit(39, 60, 1)
            inverted(true)
            pid {
                p(0.0004)
            }
            coastMode()
        }
        climberMotor.setRawOffset(MIN_CLIMB_INCHES)
        climbSetpoint = MIN_CLIMB_INCHES.inches

        GlobalScope.launch {
            periodic {
                climberCurrentEntry.setDouble(climberMotor.current)
                climberEncoderEntry.setDouble(climberEncoder.get())
                climberHeightEntry.setDouble(climberHeight.asInches)
                climbSetpointEntry.setDouble(climbSetpoint.asInches)
                relayOnEntry.setBoolean(relayOn)


                if (relayOn) {
                    relay.set(Relay.Value.kForward)
                } else {
                    relay.set(Relay.Value.kOff)
                }
            }
        }
    }

    override suspend fun default() {
        periodic {
//            climbSetpoint = (-OI.operatorController.leftThumbstickY * (MAX_CLIMB_INCHES - MIN_CLIMB_INCHES) + MIN_CLIMB_INCHES).inches
        }
    }
    override fun preEnable() {
        relayOn = false
        println("climber height $climberHeight  climber setpoint $climbSetpoint !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        climbSetpoint = climberHeight
        println("AFTER RESET:::  climber height $climberHeight  climber setpoint $climbSetpoint")
    }

    fun activateRelay() {
        relayOn = true
        println("RELAY ON!!!")
//        delay(0.5)
//        relayOn = false
//        println("waited 0.5 seconds RELAY OFF")
    }
}