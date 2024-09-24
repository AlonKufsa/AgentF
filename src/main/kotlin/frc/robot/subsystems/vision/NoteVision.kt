package frc.robot.subsystems.vision

import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import org.photonvision.PhotonCamera

object NoteVision : Sendable {
	private val camera: PhotonCamera = PhotonCamera("NOTE-Cam")
	val lastResult
		get() = camera.latestResult
	val hasTargets = lastResult.hasTargets()


	override fun initSendable(builder: SendableBuilder) {
		builder.setSmartDashboardType("NoteVision")
		builder.addBooleanProperty("Has targets", { hasTargets }, null)
		builder.addDoubleProperty("note translation",
			{ if (lastResult != null) lastResult.bestTarget.yaw else 0.0 },
			null)
	}
}