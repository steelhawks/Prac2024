package frc.robot.utils

import edu.wpi.first.networktables.*
import java.net.InetAddress

object OperatorDashboard {
    private var ntInstance: NetworkTableInstance = NetworkTableInstance.getDefault()

    private val controls: NetworkTable = ntInstance.getTable("controls")
    private val status: NetworkTable = ntInstance.getTable("status")

    // state values to send to client these SHOULD be updated in robot periodic
    var robotState: StringPublisher = status.getStringTopic("robotState").publish()
    var noteStatus: StringPublisher = status.getStringTopic("noteStatus").publish()
    var isReadyToShoot: BooleanPublisher = status.getBooleanTopic("isReadyToShoot").publish()

    init {
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
