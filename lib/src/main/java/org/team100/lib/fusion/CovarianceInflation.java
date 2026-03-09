package org.team100.lib.fusion;

import org.team100.lib.uncertainty.VariableR1;

/**
 * Covariance Inflation
 * 
 * Result variance is inverse variance weighting, with two terms for covariance
 * inflation:
 * 
 * * mean dispersion weight: reduce the influence of mean dispersion, but not to
 * zero. My eyeball-tuned value for vision updates is 0.02, i.e. 2% mean
 * dispersion to maintain responsiveness to real changes without paying too much
 * attention to noise.
 * 
 * * minimum state variance: avoid state variance collapse. My eyeball-tuned
 * value for vision updates is 9e-6, or a stddev of 0.003, i.e. state confidence
 * of a few millimeters. This seems to keep it from becoming too confident and
 * locking in on the wrong value.
 * 
 * I tuned these terms by eye, in this sheet:
 * https://docs.google.com/spreadsheets/d/1DmHL1UDd6vngmr-5_9fNHg2xLC4TEVWTN2nHZBOnje0/edit?gid=1604242948#gid=1604242948
 */
public class CovarianceInflation implements Fusor {
    private static final double CRISP_THRESHOLD = 1e-12;

    private final double m_dispersionWeight;// = 0.02;
    private final double m_minVariance;// = 0.000009;

    /**
     * @param dispersionWeight scales the effect of mean dispersion.
     * @param minSigma         the minimum result standard deviation.
     */
    public CovarianceInflation(double dispersionWeight, double minSigma) {
        m_dispersionWeight = dispersionWeight;
        m_minVariance = minSigma * minSigma;
    }

    @Override
    public VariableR1 fuse(VariableR1 a, VariableR1 b) {
        if (a.variance() < CRISP_THRESHOLD && b.variance() < CRISP_THRESHOLD) {
            return VariableR1.fromVariance(
                    (a.mean() + b.mean()) / 2,
                    Math.max((a.variance() + b.variance()) / 2, m_minVariance));
        }
        if (a.variance() < CRISP_THRESHOLD) {
            return VariableR1.fromVariance(
                    a.mean(), Math.max(a.variance(), m_minVariance));
        }
        if (b.variance() < CRISP_THRESHOLD) {
            return VariableR1.fromVariance(
                    b.mean(), Math.max(b.variance(), m_minVariance));
        }
        double wA = 1 / a.variance();
        double wB = 1 / b.variance();
        double totalWeight = wA + wB;
        double mean = (wA * a.mean() + wB * b.mean()) / totalWeight;

        // Inverse variance weight
        double variance = 1 / totalWeight;
        // Add (a little) mean dispersion, so that when very-different camera estimates
        // arrive, the state listens to them.

        variance += m_dispersionWeight * wA * Math.pow(a.mean() - mean, 2) / totalWeight
                + m_dispersionWeight * wB * Math.pow(b.mean() - mean, 2) / totalWeight;
        // Prevent variance collapse, so that the camera influence stays high
        // enough.
        variance = Math.max(variance, m_minVariance);
        return VariableR1.fromVariance(mean, variance);
    }

}
