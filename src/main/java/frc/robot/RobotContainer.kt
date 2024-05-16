package frc.robot

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
import frc.robot.commands.*
import frc.robot.subsystems.intake.IntakeSubsystem
import frc.robot.subsystems.loader.LoaderConstants
import frc.robot.subsystems.loader.LoaderSubsystem
import frc.robot.subsystems.shooter.ShooterState
import frc.robot.subsystems.shooter.ShooterSubsystem

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 *
 * In Kotlin, it is recommended that all your Subsystems are Kotlin objects. As such, there
 * can only ever be a single instance. This eliminates the need to create reference variables
 * to the various subsystems in this container to pass into to commands. The commands can just
 * directly reference the (single instance of the) object.
 */
object RobotContainer {
	val psController = CommandPS4Controller(0)

	init {
		configureBindings()
		setDefaultCommands()
	}

	/** Use this method to define your `trigger->command` mappings. */
	private fun configureBindings() {
		psController.L1().whileTrue(MaintainShooterStateCommand(ShooterState.TO_AMP))
		psController.R1().whileTrue(MaintainShooterStateCommand(ShooterState.AT_SPEAKER))

		psController.cross().toggleOnTrue(CollectAndLoadCommand())
		psController.square()
			.onTrue(TransferToShooterCommand().withTimeout(LoaderConstants.TRANSFER_TO_SHOOTER_DURATION))
		psController.triangle().onTrue(LoaderEjectToAmpCommand().withTimeout(LoaderConstants.AMP_EJECT_DURATION))
	}

	private fun setDefaultCommands() {
		IntakeSubsystem.defaultCommand = DefaultIntakeCommand()
		LoaderSubsystem.defaultCommand = DefaultLoaderCommand()
		ShooterSubsystem.defaultCommand = DefaultShooterCommand()
	}

	fun getAutonomousCommand(): Command? {
		// TODO: Implement properly
		return null
	}
}