package frc.robot

import com.hamosad1657.lib.units.radToDeg
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Translation2d
import org.junit.jupiter.api.Test

import org.junit.jupiter.api.Assertions.*
import kotlin.math.atan

internal class ExampleTest
{
    // To learn more about how to write unit tests, see the
    // JUnit 5 User Guide at https://junit.org/junit5/docs/current/user-guide/

    @Test
    fun translation2DCoordinatesCheck()
    {
        assertEquals(MathUtil.inputModulus(Translation2d(5.0, -7.0).angle.degrees, 0.0, 360.0), 360.0-(radToDeg(atan(7.0/5.0))), 1.0)
        println(Translation2d(5.0, -7.0).angle.degrees)
        println(360.0-(radToDeg(atan(7.0/5.0))))
    }
}

