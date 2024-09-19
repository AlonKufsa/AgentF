package frc.robot.subsystems.vision

import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.photonvision.PhotonCamera
import org.photonvision.targeting.PhotonPipelineResult

object NoteVision : SubsystemBase("Note vision") {

	val camera: PhotonCamera = PhotonCamera("NOTE-Cam")

	val lastResult: PhotonPipelineResult
		get() = camera.latestResult


	override fun initSendable(builder: SendableBuilder) {
		builder.addBooleanProperty("Is note detected", { lastResult.hasTargets() }, null)
	}
}