package org.team100.lib.subsystems.five_bar.setups;

import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.subsystems.five_bar.FiveBarServo;
import org.team100.lib.visualization.FiveBarVisualization;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SetupServo implements Runnable {
    private final FiveBarServo m_fiveBar;
    private final FiveBarVisualization m_viz;

    public SetupServo() {
        final Logging logging = Logging.instance();
        final LoggerFactory logger = logging.rootLogger;
        TotalCurrentLog currentLog = new TotalCurrentLog(logger);
        XboxController controller = new XboxController(0);

        m_fiveBar = new FiveBarServo(logger, currentLog);
        m_viz = new FiveBarVisualization(m_fiveBar::getJointPositions);
        m_fiveBar.setDefaultCommand(m_fiveBar.position(
                controller::getLeftX, controller::getRightX));

        // These bindings are remembered by the trigger event loop, so we don't need to
        // retain them.
        new Trigger(controller::getAButton).whileTrue(m_fiveBar.home());
        new Trigger(controller::getBButton).onTrue(m_fiveBar.zero());
    }

    @Override
    public void run() {
        m_viz.periodic();
    }

}
