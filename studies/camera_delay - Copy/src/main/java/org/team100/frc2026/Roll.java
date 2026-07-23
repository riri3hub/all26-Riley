package org.team100.frc2026;

import java.util.function.ObjDoubleConsumer;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.Rotation2dLogger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

/**
 * Accepts transforms (from camera blips), extracts the roll component, and
 * pushes it to the delegate.
 */
public class Roll implements ObjDoubleConsumer<Transform3d> {
    private final Rotation2dLogger m_logRoll;
    private final ObjDoubleConsumer<Rotation2d> m_delegate;

    public Roll(LoggerFactory parent,
            ObjDoubleConsumer<Rotation2d> delegate) {
        LoggerFactory log = parent.type(this);
        m_logRoll = log.rotation2dLogger(Level.TRACE, "roll");
        m_delegate = delegate;
    }

    @Override
    public void accept(Transform3d t, double value) {
        Rotation2d roll = new Rotation2d(t.getRotation().getX());
        m_logRoll.log(() -> roll);
        m_delegate.accept(roll, value);
    }

}
