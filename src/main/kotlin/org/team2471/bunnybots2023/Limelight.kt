package org.team2471.bunnybots2023

import edu.wpi.first.math.filter.LinearFilter
import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.bunnybots2023.Limelight.bucketWidth
import org.team2471.bunnybots2023.Limelight.cl
import org.team2471.bunnybots2023.Limelight.limeLightToTarget
import org.team2471.bunnybots2023.Limelight.limelightScreenWidth
import org.team2471.bunnybots2023.Limelight.limelightStaticAngle
import org.team2471.bunnybots2023.Limelight.targetTopHeight
import org.team2471.bunnybots2023.Limelight.totalLatency
import org.team2471.bunnybots2023.Limelight.vBotCentCoordsLengthEntry
import org.team2471.bunnybots2023.Limelight.vBotCentCoordsXEntry
import org.team2471.bunnybots2023.Limelight.vBotCentCoordsYEntry
import org.team2471.bunnybots2023.Limelight.vMax
import org.team2471.bunnybots2023.Limelight.vMin
import org.team2471.frc.lib.coroutines.MeanlibDispatcher
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.units.Angle
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.units.feet
import org.team2471.frc.lib.math.Vector2
import org.team2471.frc.lib.units.*
import org.team2471.frc.lib.units.Angle.Companion.tan
import kotlin.math.pow

object Limelight : Subsystem("Limelight") {
    private val datatable = NetworkTableInstance.getDefault().getTable("limelight-front")
    private val validTargetsEntry = datatable.getEntry("tv")
    private val tl = datatable.getEntry("tl")
    private val cl = datatable.getEntry("cl")
    val ledModeEntry = datatable.getEntry("ledMode")

    private val table = NetworkTableInstance.getDefault().getTable("Limelight")
    val vBotCentCoordsXEntry: NetworkTableEntry = table.getEntry("vBotCentCoords X")
    val vBotCentCoordsYEntry: NetworkTableEntry = table.getEntry("vBotCentCoords Y")
    val vBotCentCoordsLengthEntry: NetworkTableEntry = table.getEntry("vBotCentCoords Length")
//    val vAngleEntry: NetworkTableEntry = table.getEntry("vAngle")
    val botCentCoordsXEntry: NetworkTableEntry = table.getEntry("botCentCoords X")
    val botCentCoordsYEntry: NetworkTableEntry = table.getEntry("botCentCoords Y")
    val targetDistEntry: NetworkTableEntry = table.getEntry("Target Distance (Inches)")
    val targetAngleEntry: NetworkTableEntry = table.getEntry("Target Angle (Degrees)")


    val advantageBucketPose = table.getEntry("Advantage Bucket Pose")

    val totalLatency: Double
        get() = cl.getDouble(0.0) + tl.getDouble(0.0)

    private const val lengthHeightMinRatio = 2.5
    val limelightHeight = 32.inches
    val targetElevation = 42.inches
    val targetHeight: Length
        get() = if (AutoChooser.redSide) 10.inches else 12.inches
    val targetTopHeight = targetElevation + targetHeight
    val limeLightToTarget = targetTopHeight - limelightHeight
    val limelightStaticAngle = 25.7.degrees
    const val limelightScreenWidth = 320
    const val limelightScreenHeight = 320
    const val bucketWidth = 10 + 2/8 // Inches
    const val vMax = 20.0 /* feet per second */ /50.0
    const val vMin = 2.0 /* feet per second */ /50.0
    val botCentFilterX = LinearFilter.movingAverage(5)
    val botCentFilterY = LinearFilter.movingAverage(5)

    var enemyBuckets : List<BucketTarget> = arrayListOf<BucketTarget>()

    var lastJoystickTarget: Angle = 0.0.degrees

    // field centric
    val limelightAngle : Angle
        get() = Turret.turretAngle + Drive.heading


    init {

        GlobalScope.launch(MeanlibDispatcher) {

            periodic {
                val prevEnemyBuckets = enemyBuckets
                enemyBuckets = identifyBuckets()
//                println("HI: ${enemyBuckets.isNotEmpty()}")
                if (enemyBuckets.isNotEmpty()) {
//                    for (target in enemyBuckets) {
//                        println("hi: ${(DriverStation.getAlliance().name)}")
//                    }
//                    for (i in filteredTargets!!) {
//                        println("ID: ${i.id}, IsRed: ${i.isRed} IsValid: ${i.isRed == FieldManager.isBlueAlliance}")
//                    }

                    enemyBuckets = enemyBuckets.filter {
                        it.isRed != (DriverStation.getAlliance().name.equals("Red"))
                    }
                    if (prevEnemyBuckets.isNotEmpty() && enemyBuckets.isNotEmpty()) {
                        println("hi2: ${enemyBuckets[0].isRed}")
                        enemyBuckets[0].prevTarget = prevEnemyBuckets[0]
                        vBotCentCoordsLengthEntry.setDouble(enemyBuckets[0].vBotCentCoords.length * 50)
                        botCentCoordsXEntry.setDouble(enemyBuckets[0].botCentCoords.x)
                        botCentCoordsYEntry.setDouble(enemyBuckets[0].botCentCoords.y)
                        targetDistEntry.setDouble(enemyBuckets[0].dist.asInches)
                        targetAngleEntry.setDouble(enemyBuckets[0].angle.asDegrees)
//                        println(Angle.atan(enemyBuckets[0].y * (22.85).degrees.tan()))

//                        println(enemyBuckets[0].botCentCoords)
                    }
//                    println(enemyBuckets[0].prevTarget)
                }

                validTargetsEntry.setBoolean(seesTargets)
            }
        }
    }

    fun Angle.toFieldCentric() : Angle {
        return this + Drive.heading
    }

    fun Angle.toRobotCentric() : Angle {
        return this - Drive.heading
    }

    val seesTargets: Boolean
        get() = enemyBuckets.isNotEmpty()

    // gets a bucket in field-centric bounds
    fun getBucketInBounds(upperBound: Angle, lowerBound: Angle) : BucketTarget? {
        var foundTarget : BucketTarget? = null
        for (target in enemyBuckets.indices) {
            val angleToBucket : Angle = enemyBuckets[target].angle.unWrap((upperBound + lowerBound)/2.0)
            if (lowerBound < angleToBucket &&
                angleToBucket < upperBound) {
                foundTarget = enemyBuckets[target]
                break
            }
        }
        return foundTarget
    }

    fun identifyBuckets(): List<BucketTarget> {

        // find all long strips
        var longStrips = arrayListOf<Int>()
        for (entryNum in 0..7) {
            //println(datatable.getEntry("thor${entryNum}").getDouble(0.0) / datatable.getEntry("tvert${entryNum}").getDouble(0.0))
            if (datatable.getEntry("thor${entryNum}").getDouble(0.0) / datatable.getEntry("tvert${entryNum}").getDouble(0.0) >= lengthHeightMinRatio) {
                longStrips.add(entryNum)
            }
        }
//        println("LongStrips: ${longStrips}")
        // find color of long strips
        var longStripsColor = IntArray(longStrips.size) {0}
        for (entryNum in 0 .. 7) {
            if (longStrips.contains(entryNum)) continue
            if (datatable.getEntry("ta${entryNum}").getDouble(0.0) == 0.0) continue

            val shortStripX = datatable.getEntry("tx${entryNum}").getDouble(0.0)
            val shortStripY = datatable.getEntry("ty${entryNum}").getDouble(0.0)

            for (i in 0 until longStrips.size) {
                val target = longStrips[i]
                val longStripX = datatable.getEntry("tx${target}").getDouble(0.0)
                val longStripHorizontal = datatable.getEntry("thor${target}").getDouble(0.0) / (0.5 * limelightScreenWidth)
                val longStripY = datatable.getEntry("ty${target}").getDouble(0.0)
                if (shortStripX < longStripX + longStripHorizontal/2 &&
                    shortStripX > longStripX - longStripHorizontal/2) {
                    if (shortStripY > longStripY) {
                        longStripsColor[i] -= 1
                    } else {
                        longStripsColor[i] += 1
                    }
                }
//                println("X: $shortStripX $longStripX $longStripHorizontal")
            }
        }


        var targets = arrayListOf<BucketTarget>()
        for (i in 0 until longStrips.size) {
//            println("id: ${longStrips[i]}: " + longStripsColor[i])
            if (longStripsColor[i] == 0) continue // color for long strip is not known
            targets.add(BucketTarget(
                longStrips[i],
                longStripsColor[i] > 0,
                datatable.getEntry("tx${longStrips[i]}").getDouble(0.0),
                datatable.getEntry("ty${longStrips[i]}").getDouble(0.0),
                datatable.getEntry("thor${longStrips[i]}").getDouble(0.0),
                datatable.getEntry("tvert${longStrips[i]}").getDouble(0.0)
            ))
        }
//
//        for (target in targets) {
//            println("Target ${target.id} angleWidth: ${target.angleWidth} ")
//        }

        return targets
    }
    fun targetNum(): Int {
        var amount: Int = 0
        for (entryNum in 0..7) {
            if (datatable.getEntry("ta${entryNum}").getDouble(0.0) != 0.0) {
                println("found tag ta${entryNum}")
                amount += 1
            }
        }
        return amount
    }

    fun toggleLight() {
        if (ledModeEntry.getDouble(0.0).toInt() == 1) {
            println("turning on limelight")
            ledModeEntry.setDouble(0.0)
        } else {
            println("turning off limelight")
            ledModeEntry.setDouble(1.0)
        }
    }

}
data class BucketTarget (
    val id: Int,
    val isRed: Boolean,
    val x: Double,
    val y: Double,
    val pixelWidth: Double,
    val pixelHeight: Double
) {
    var prevTarget: BucketTarget? = null
        set(value) {
            if (value != null) {
                vBotCentCoordsXEntry.setDouble(vBotCentCoords.times(50.0).x)
                vBotCentCoordsYEntry.setDouble(vBotCentCoords.times(50.0).y)
//                println("VANGLE!!!! :$vAngle")
            } else {
//                println("Oh No!")
            }
            field = value
        }
    val angle = Limelight.limelightAngle + Angle.atan(x * (29.8).degrees.tan())
    val vertTopAngle = limelightStaticAngle + Angle.atan((y /*+ (pixelHeight / 2)*/) * (22.85).degrees.tan())
    val dist = limeLightToTarget / tan(vertTopAngle)
    val botCentCoords = Vector2((angle.sin()*dist.asFeet), (angle.cos()*dist.asFeet))
    val vBotCentCoords: Vector2
        get() {
            return botCentCoords - (prevTarget?.botCentCoords ?: botCentCoords)
        }
    val vDist: Length
        get() {
            return dist - (prevTarget?.dist ?: dist)
        }



    val ticksToTarget: Double //Given to you by desmos nonlinear regression!
        get() {
            var x = dist.asInches
//            return 14.109 * kotlin.math.tan(0.013571 * x)
            //                                                                                    ↓ this is limelight latency! keep this
            return (0.0000453727 * x.pow(3)) - (0.00233868 * x.pow(2)) + (0.229364 * x) + ((totalLatency / 1000) * 50)
        }

    fun pBotCentCoords(ticks: Double): Vector2{

        if (vBotCentCoords.length in vMin..vMax){
//            println(vBotCentCoords.times(ticks.toDouble()))
//            println("${botCentCoords.round(2)} + ${(vBotCentCoords * ticks.toDouble()).round(2)} = ${(botCentCoords + (vBotCentCoords * ticks.toDouble())).round(2)}")
            return botCentCoords + (vBotCentCoords * ticks.toDouble())
        }
        else{

//            println("Oh nooooo!${vBotCentCoords.length * 50}")
            return botCentCoords
        }

//        return botCentCoords + (vBotCentCoords.times(ticks.toDouble()))
    }
}
