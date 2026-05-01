package org.team100.lib.subsystems.tank.commands;

import java.util.function.DoubleSupplier;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.subsystems.tank.TankDrive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Manual tank-drive control using a single joystick (if using an
 * xbox style control, this will be the right-hand stick).
 */
public class TankManual extends Command {
    private final DoubleSupplier m_translation;
    private final DoubleSupplier m_rotation;
    private final double m_maxV;
    private final double m_maxOmega;
    private final TankDrive m_drive;
    private final DoubleLogger m_logTranslation;
    private final DoubleLogger m_logRotation;

    public TankManual(
            LoggerFactory parent,
            DoubleSupplier translation,
            DoubleSupplier rotation,
            double maxV,
            double maxOmega,
            TankDrive robotDrive) {
        LoggerFactory log = parent.type(this);
        m_logTranslation = log.doubleLogger(Level.TRACE, "translation");
        m_logRotation = log.doubleLogger(Level.TRACE, "rotation");
        m_translation = translation;
        m_rotation = rotation;
        m_maxV = maxV;
        m_maxOmega = maxOmega;
        m_drive = robotDrive;
        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        double translationM_S = MathUtil.applyDeadband(m_translation.getAsDouble(), 0.1) * m_maxV;
        double rotationRad_S = MathUtil.applyDeadband(m_rotation.getAsDouble(), 0.1) * m_maxOmega;
        m_logTranslation.log(() -> translationM_S);
        m_logRotation.log(() -> rotationRad_S);
        m_drive.setVelocity(translationM_S, rotationRad_S);
    }
}
