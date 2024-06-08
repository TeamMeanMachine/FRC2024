package org.team2471.frc2024

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DriverStation
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
import org.team2471.frc.lib.util.Timer

object Climb: Subsystem("Climb") {
    private val table = NetworkTableInstance.getDefault().getTable("Climb")

    private val climberCurrentEntry = table.getEntry("Climb Current")
    private val climberEncoderEntry = table.getEntry("Climb Encoder Value")
    private val climberHeightEntry = table.getEntry("Motor Height")
    private val climbSetpointEntry = table.getEntry("Climb Setpoint")
    private val relayOnEntry = table.getEntry("Relay On")
    private val matchTimeLeft = table.getEntry("Match time left")

    val climberMotor = MotorController(SparkMaxID(Sparks.CLIMBER, "Climb/Climb"))
    val climberEncoder = DutyCycleEncoder(DigitalSensors.CLIMBER)
    private val relay = Relay(Solenoids.CLIMB_SWITCH)

    var preEnabled = false

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


    private var relayOn: Boolean = false

    const val MIN_CLIMB_INCHES = 23.5
    const val MAX_CLIMB_INCHES = 45.625

    init {
        climberMotor.config {
            feedbackCoefficient = 0.4331 * 1.1538//1.0/*23.68 * 0.0985 / 5.337*/ //need ticks (or rotations) per inch and maybe gear ratio
            currentLimit(39, 60, 1)
            inverted(true)
            pid {
                p(0.0006)
            }
            coastMode()
        }
        climberMotor.setRawOffset(MIN_CLIMB_INCHES)
        climbSetpoint = MIN_CLIMB_INCHES.inches
        val matchTimeTimer = Timer()

        GlobalScope.launch {
            periodic {
                if (DriverStation.getMatchTime() < 1.0 && DriverStation.getMatchTime() > 0.0 && !Robot.isAutonomous) {
                    println("locking climber!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Match Time ${DriverStation.getMatchTime()}")
                    relayOn = true
                }

                climbSetpointEntry.setDouble(climbSetpoint.asInches)
                climberEncoderEntry.setDouble(climberEncoder.get())
                climberHeightEntry.setDouble(climberHeight.asInches)
                relayOnEntry.setBoolean(relayOn)
                matchTimeLeft.setDouble(DriverStation.getMatchTime())


                if (relayOn && preEnabled) {
                    relay.set(Relay.Value.kForward)
//                    println("SETTING RELAY TO TRUE!! periodic")
                } else {
                    relay.set(Relay.Value.kOff)
                }


            }
        }
    }

    override suspend fun default() {
        periodic {
            climberCurrentEntry.setDouble(climberMotor.current)
        }
    }
    override fun preEnable() {
        GlobalScope.launch {
            relayOn = false
            preEnabled = true
//        println("climber height $climberHeight  climber setpoint $climbSetpoint !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            climbSetpoint = climberHeight
//        println("AFTER RESET:::  climber height $climberHeight  climber setpoint $climbSetpoint")
        }
    }
    override fun onDisable() {
        preEnabled = false
    }

    fun activateRelay() {
        relayOn = !relayOn
        println("RELAY $relayOn!!!")
    }
}