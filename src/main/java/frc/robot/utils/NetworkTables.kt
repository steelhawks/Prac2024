package frc.robot.utils

import edu.wpi.first.networktables.*
import java.net.InetAddress

object NetworkTables {
    private var ntInstance: NetworkTableInstance = NetworkTableInstance.getDefault()

    var robotState: StringPublisher
    var noteStatus: StringPublisher
    var isReadyToShoot: BooleanPublisher

    private val controls: NetworkTable = ntInstance.getTable("controls")
    private val status: NetworkTable = ntInstance.getTable("status")

    init {
        robotState = status.getStringTopic("robotState").publish()
        noteStatus = status.getStringTopic("noteStatus").publish()
        isReadyToShoot = status.getBooleanTopic("isReadyToShoot").publish()

        try {
            val ip = InetAddress.getLocalHost()
            println("IP ADDR NT4 " + ip.hostAddress + ":5810")
        } catch (e: Exception) {
            e.printStackTrace()
        }
    }

    fun close() {
        robotState.close()
        noteStatus.close()
    }
}