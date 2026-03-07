package org.team100.frc2026;

import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.DoubleFunction;

import org.team100.lib.targeting.FiringParameters;
import org.team100.lib.util.InterpolatingMap100;
import org.team100.lib.util.Relation;

/**
 * Interpolates shooting parameters, given range in meters.
 */
public class ShooterTable {
    private final Relation<FiringParameters> m_table;
    private final DoubleFunction<Double> m_rangeToSpeed;
    private final DoubleFunction<Double> m_rangeToElevation;
    private final DoubleFunction<Double> m_rangeToTof;
    private final InterpolatingMap100<Double, FiringParameters> m_rangeMap;

    /**
     * Returns empty for out-of-range, so if you want to be sure to return
     * something, include "far away" points in the list.
     */
    public ShooterTable(List<FiringParameters> rows) {
        m_table = new Relation<>();
        m_table.addAll(rows);
        m_rangeToSpeed = m_table.function(
                FiringParameters::range, FiringParameters::speed);
        m_rangeToElevation = m_table.function(
                FiringParameters::range, FiringParameters::elevation);
        m_rangeToTof = m_table.function(
                FiringParameters::range, FiringParameters::tof);
        m_rangeMap = m_table.map(FiringParameters::range,
                FiringParameters::interpolate);
    }

    public OptionalDouble angle(double rangeM) {
        return emptyIfNull(m_rangeToElevation.apply(rangeM));
    }

    public OptionalDouble speed(double rangeM) {
        return emptyIfNull(m_rangeToSpeed.apply(rangeM));
    }

    public OptionalDouble tof(double rangeM) {
        return emptyIfNull(m_rangeToTof.apply(rangeM));
    }

    public Optional<FiringParameters> forRange(double rangeM) {
        return emptyIfNull(m_rangeMap.get(rangeM));
    }

    /////////////////////////////////////////////////

    private OptionalDouble emptyIfNull(Double x) {
        if (x == null)
            return OptionalDouble.empty();
        return OptionalDouble.of(x);
    }

    private <V> Optional<V> emptyIfNull(V v) {
        if (v == null)
            return Optional.empty();
        return Optional.of(v);
    }
}