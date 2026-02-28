package org.team100.frc2026.robot;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.print;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;

public class Tester {

    private static final String TEST = "TEST: ";
    private final Machinery m_machinery;
    private final Alert m_alert;

    public Tester(Machinery machinery) {
        m_machinery = machinery;
        m_alert = new Alert(text("waiting"), AlertType.kInfo);
        m_alert.set(true);
    }

    /**
     * TEST ALL MOVEMENTS
     * 
     * For pre- and post-match testing.
     * 
     * Enable "test" mode and press driver "a" and "b" together.
     * (in simulation this is buttons 1 and 2, or "z" and "x" on the keyboard)
     * 
     * DANGER DANGER DANGER DANGER DANGER DANGER DANGER DANGER
     *
     * THIS WILL MOVE THE ROBOT VERY FAST!
     *
     * DO NOT RUN with the wheels on the floor!
     *
     * DO NOT RUN without tiedown clamps.
     *
     * DANGER DANGER DANGER DANGER DANGER DANGER DANGER DANGER
     */
    public Command prematch() {
        return sequence(
                show("*** WARNING! MOTION STARTS IN 4 SECONDS ***"),
                m_machinery.m_beeper.start(),
                show("ahead slow"),
                m_machinery.m_beeper.progress(),
                m_machinery.m_drive.aheadSlow().withTimeout(1),
                m_machinery.m_drive.stopCommand(),
                show("rightward slow"),
                m_machinery.m_beeper.progress(),
                m_machinery.m_drive.rightwardSlow().withTimeout(1),
                m_machinery.m_drive.stopCommand(),
                show("done"),
                m_machinery.m_beeper.done())
                .withName("test all movements");
    }

    /** Show the text as an alert and on the console. */
    private Command show(String text) {
        return parallel(
                runOnce(() -> {
                    m_alert.setText(text(text));
                    m_alert.set(true);
                }),
                print(text(text)));
    }

    private String text(String text) {
        return TEST + text;
    }

}
