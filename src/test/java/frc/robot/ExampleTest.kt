package frc.robot

import edu.wpi.first.math.geometry.Translation2d
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Test

internal class ExampleTest {
	// To learn more about how to write unit tests, see the
	// JUnit 5 User Guide at https://junit.org/junit5/docs/current/user-guide/

	@Test
	fun translation2DCoordinatesCheck() {
		assertEquals(Translation2d(-1.0, 0.0).angle.degrees, 0.0)
	}
}

