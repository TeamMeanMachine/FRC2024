@file:Suppress("unused")

package org.team2471.bunnybots2023

object Talons {
    const val INTAKE_FRONT_LEADER = 13
    const val INTAKE_FRONT_FOLLOWER = 18
    const val INTAKE_CENTER_LEFT = 20
    const val INTAKE_CENTER_RIGHT = 14
    const val HOPPER_UPTAKE = 8
    const val HOPPER_CONVEYOR = 7
    const val SHOOTER_ONE = 2
    const val SHOOTER_TWO = 15
}


object Sparks {
    const val FRONT_LEFT_STEER = 17
    const val FRONT_LEFT_DRIVE = 19
    const val FRONT_RIGHT_STEER = 16
    const val FRONT_RIGHT_DRIVE = 26
    const val REAR_RIGHT_STEER = 6
    const val REAR_RIGHT_DRIVE = 21
    const val REAR_LEFT_STEER = 5
    const val REAR_LEFT_DRIVE = 3
}

object AnalogSensors {
    const val SHOOTER_ENCODER = 99
    const val TURRET_ENCODER = 0
}

object DigitalSensors {
    const val REAR_LEFT = 6
    const val REAR_RIGHT = 5
    const val FRONT_RIGHT = 4
    const val FRONT_LEFT = 3

    const val HOPPER_LOW = 9
    const val HOPPER_HIGH = 8
}

object Falcons {
    const val TURRET_ONE = 35
    const val TURRET_TWO = 36
}

object CANCoders {

}

object Solenoids {
    const val INTAKE = 0
}

object OtherCAN {
    const val PNEUMATIC_HUB = 22
}