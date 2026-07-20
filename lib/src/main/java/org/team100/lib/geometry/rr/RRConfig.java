package org.team100.lib.geometry.rr;

/**
 * Joint configuration for the RR example.
 * 
 * @param q1 rotation of joint 1 ("proximal", "shoulder") ,CCW rad from x
 * @param q2 rotation of joint 2 ("distal", "elbow"),CCW rad from link 1
 */
public record RRConfig(double q1, double q2) {

}
