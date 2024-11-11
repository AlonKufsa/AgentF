package frc.robot

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.auto.NamedCommands
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
import frc.robot.commands.*
import frc.robot.subsystems.intake.IntakeSubsystem
import frc.robot.subsystems.leds.LedSubsystem
import frc.robot.subsystems.loader.LoaderConstants
import frc.robot.subsystems.loader.LoaderSubsystem
import frc.robot.subsystems.shooter.ShooterState
import frc.robot.subsystems.shooter.ShooterSubsystem
import frc.robot.subsystems.swerve.SwerveSubsystem
import frc.robot.vision.NoteVision

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
	private val mainController = CommandPS4Controller(1)

	init {
		configureBindings()
		setDefaultCommands()
		sendSubsystemInfo()
	}

	/** Use this method to define your `trigger->command` mappings. */
	private fun configureBindings() {

		mainController.L1().whileTrue(MaintainShooterStateCommand(ShooterState.TO_AMP, true))
		mainController.R1().whileTrue(MaintainShooterStateCommand(ShooterState.AT_SPEAKER, true))

		mainController.cross().toggleOnTrue(CollectAndLoadCommand())
		mainController.square()
			.toggleOnTrue(TransferToShooterCommand(false).withTimeout(LoaderConstants.TRANSFER_TO_SHOOTER_DURATION))
		mainController.triangle()
			.toggleOnTrue(LoaderEjectToAmpCommand().withTimeout(LoaderConstants.AMP_EJECT_DURATION))

		mainController.L3()
			.toggleOnTrue(ManualShootingAngleControlCommand({ mainController.leftX }, { mainController.leftY }))

		mainController.options().whileTrue(resetGyroCommand())

		mainController.share().onTrue(ShootLoadedNoteCommand(ShooterState.AT_SPEAKER))

		mainController.circle().whileTrue(AssistedIntakeCommand({ mainController.leftY }))
	}

	private fun setDefaultCommands() {
		IntakeSubsystem.defaultCommand = DefaultIntakeCommand()
		LoaderSubsystem.defaultCommand = DefaultLoaderCommand()
		SwerveSubsystem.defaultCommand =
			FieldRelativeDriveCommand({ mainController.leftY }, { -mainController.leftX }, { -mainController.rightX })
		ShooterSubsystem.defaultCommand = DefaultShooterCommand()
//		ClimbingSubsystem.defaultCommand =
//			DefaultClimbingCommand({ mainController.leftY }, { mainController.rightX })
	}

	private fun sendSubsystemInfo() {
		SmartDashboard.putData(ShooterSubsystem)
		SmartDashboard.putData(LoaderSubsystem)
		SmartDashboard.putData(IntakeSubsystem)
		SmartDashboard.putData(LedSubsystem)
		SmartDashboard.putData(SwerveSubsystem)
		SmartDashboard.putData(NoteVision)
	}

	private val autoChooser = AutoBuilder.buildAutoChooser("TestAuto")

	init {
		SmartDashboard.putData("Auto chooser", autoChooser)
	}

	// Autonomous
	init {
		NamedCommands.registerCommand("Shoot", ShootLoadedNoteCommand(ShooterState.AT_SPEAKER))
		NamedCommands.registerCommand("Intake", CollectAndLoadCommand())
	}

	fun getAutonomousCommand(): Command {
		return autoChooser.selected
	}
}