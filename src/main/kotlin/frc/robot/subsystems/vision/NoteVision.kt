package frc.robot.subsystems.vision

import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.photonvision.PhotonCamera

object NoteVision : SubsystemBase("Note vision") {

	private val camera: PhotonCamera = PhotonCamera("NOTE-Cam")
	val lastResult
		get() = camera.latestResult
	val hasTargets = lastResult.hasTargets()


	override fun initSendable(builder: SendableBuilder) {
		builder.addDoubleProperty("note translation",
			{ if (lastResult != null) lastResult.bestTarget.yaw else 0.0 },
			null)
	}
}