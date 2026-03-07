package org.team100.frc2026;

import java.util.List;
import java.util.OptionalDouble;
import java.util.function.Supplier;

import org.team100.frc2026.field.FieldConstants2026;
import org.team100.lib.targeting.FiringParameters;

import edu.wpi.first.math.geometry.Translation2d;

/** Chooses where to aim, supplies parameters. */
public class Targeter {
    private final Supplier<Translation2d> m_position;
    /** High shots to score */
    private final ShooterTable m_score;
    /** Low shots to save power */
    private final ShooterTable m_lob;

    public Targeter(Supplier<Translation2d> position) {
        m_position = position;
        m_score = new ShooterTable(
                List.of(
                        new FiringParameters(0, 5, 1, 0.1),
                        new FiringParameters(1.49, 5, 0.90, 0.1),
                        new FiringParameters(2.07, 6, 0.78, 0.2),
                        new FiringParameters(2.50, 7, 0.66, 0.3),
                        new FiringParameters(3.02, 8, 0.59, 0.4),
                        new FiringParameters(3.59, 9, 0.53, 0.5),
                        new FiringParameters(4.10, 10, 0.48, 0.6),
                        new FiringParameters(4.50, 11, 0.44, 0.7),
                        new FiringParameters(16, 12, 0.5, 2)));
        m_lob = new ShooterTable(
                List.of(
                        new FiringParameters(0.0, 4, 0.3, 0.3),
                        new FiringParameters(1.0, 5, 0.3, 0.3),
                        new FiringParameters(2.0, 6, 0.3, 0.3),
                        new FiringParameters(3.0, 7, 0.3, 0.3),
                        new FiringParameters(4.0, 8, 0.3, 0.3),
                        new FiringParameters(5.0, 9, 0.3, 0.3),
                        new FiringParameters(6.0, 10, 0.3, 0.3),
                        new FiringParameters(16, 12, 0.5, 2)));
    }

    public OptionalDouble angle() {
        Translation2d p = m_position.get();
        if (FieldConstants2026.isInAllianceZone(p)) {
            return m_score.angle(FieldConstants2026.rangeToHub(p));
        }
        if (FieldConstants2026.isInNeutralZone(p)) {
            return m_lob.angle(FieldConstants2026.rangeToLob(p));
        }
        return OptionalDouble.empty();
    }

    public OptionalDouble speed() {
        Translation2d p = m_position.get();
        if (FieldConstants2026.isInAllianceZone(p)) {
            return m_score.speed(FieldConstants2026.rangeToHub(p));
        }
        if (FieldConstants2026.isInNeutralZone(p)) {
            return m_lob.speed(FieldConstants2026.rangeToLob(p));
        }
        return OptionalDouble.empty();
    }

}
