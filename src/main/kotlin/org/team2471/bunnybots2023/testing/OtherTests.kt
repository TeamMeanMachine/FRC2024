package org.team2471.bunnybots2023.testing

import org.team2471.bunnybots2023.OI
import org.team2471.bunnybots2023.Turret
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.use


suspend fun turretOITest() = use(Turret){
    periodic {
        val power = OI.operatorController.rightThumbstickX / 4.0
        Turret.turningMotor.setPercentOutput(1.0)
        println("power = $power")
    }
}