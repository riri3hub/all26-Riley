package org.team100.lib.motor.ctre;

import org.team100.lib.config.SimpleDynamics;
import org.team100.lib.config.Friction;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode100;
import org.team100.lib.util.CanId;

/**
 * Falcon 500 using Phoenix 6.
 * 
 * @see https://store.ctr-electronics.com/content/datasheet/Motor%20Performance%20Analysis%20Report.pdf
 */
public class Falcon500Motor extends Talon6Motor {

    public Falcon500Motor(
            LoggerFactory parent,
            TotalCurrentLog currentLog,
            CanId canId,
            NeutralMode100 neutral,
            MotorPhase motorPhase,
            double supplyLimit,
            double statorLimit,
            SimpleDynamics ff,
            Friction friction,
            PIDConstants pid) {
        super(parent, currentLog, canId, neutral, motorPhase, supplyLimit,
                statorLimit, ff, friction, pid);
    }

    @Override
    public double kROhms() {
        return 0.03;
    }

    @Override
    public double kTNm_amp() {
        return 0.018;
    }

    @Override
    public double kFreeSpeedRPM() {
        return 6079;
    }

}
