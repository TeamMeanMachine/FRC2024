package org.team2471.frc2024

import edu.wpi.first.networktables.NetworkTableInstance
import org.team2471.frc.lib.actuators.MotorController
import org.team2471.frc.lib.actuators.SparkMaxID
import org.team2471.frc.lib.framework.Subsystem

object Climber: Subsystem("Climber") {
    private val table = NetworkTableInstance.getDefault().getTable("")

    private val climberPercentEntry = table.getEntry("Intake Percent")
    private val climberCurrentEntry = table.getEntry("Intake Current")

    private val climberMotor = MotorController(SparkMaxID(Sparks.CLIMBER))

    // Don't know how climber works so imma just stop here
}