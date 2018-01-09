import frc.team5987.robot.subsystems.Elevator;
import org.junit.*;

import java.io.IOException;
import java.io.PrintWriter;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class ElevatorTest {

    //Torque in N m
    static final double kStallTorque = 2.402;
    //Current in Amps
    static final double kStallCurrent = 131;
    static final double kFreeCurrent = 2.70;
    //Speed in RPM
    static final double kFreeSpeed = 5330;
    //Mass of the Elevator
    static final double kMass = 20.0;

    static final int kNumMotors = 2;
    //Resistance of motor (using Ohm's law)
    static final double kResistence = 12.0/kStallCurrent;
    //Calculating kV (generator half of EQ)
    static final double Kv = ((kFreeSpeed/60.0*2.0*Math.PI) / (12.0-kResistence*kFreeCurrent));

    //Torque constant
    static final double Kt = (kNumMotors*kStallTorque)/kStallCurrent;
    //Gear ratio
    static final double kG = 5;
    //Pulley radius
    static final double kr = 17 * 0.25 * 0.254 / Math.PI / 2.0;
    //Control loop time step (in seconds)a
    static final double kDt = 0.010;

    //V = I*r + omega/Kv
    //torque = Kt * I

    private double GetAcceleration(double voltage) {
        return -Kt*kG*kG / (Kv * kResistence * kr * kr * kMass) * velocity +
                kG * Kt /(kResistence * kr * kMass) * voltage;
    }

    double velocity = 0;
    double position = 0.2;
    double voltage = 0;
    double offset = 0.2;
    Elevator elevator;

    double encoder() {
        return position + offset;
    }
    boolean hallEffect() {
        return position < 0.0 && position > -0.01;
    }
    double currentTime = 0.0;

    void SimulateTime(double time, double voltage) {
        double startingTime = time;
        while (time > 0) {
            double current_dt = Math.min(time, 0.001);
            position += current_dt * velocity;
            velocity += current_dt * GetAcceleration(voltage);
            time -= 0.001;
            assertTrue("Subsystem should not exceed max height",position< Elevator.kMaxHeight);
            assertTrue("Subsystem should not exceed min height", position > Elevator.kMinHeight);
        }
        currentTime += startingTime;
    }

    PrintWriter writer;

    @Test
    public void moveToPositionAfterZeroing() throws IOException{
        elevator = new Elevator();
        elevator.setGoal(1);
        writer = new PrintWriter("simulationplot.txt","UTF-8");
        writer.println("# time\tvoltage\tposition\thalleffect\tstate\tgoal");
        for (int i = 0; i < 700; i++) {
            voltage = elevator.update(encoder(), hallEffect(), true);
            if (elevator.state == Elevator.State.RUNNING) {
                assertTrue("Voltage cannot exceed 12V",voltage <= 12.0);
                assertTrue("Voltage cannot exceed -12V",voltage >= -12.0);
            } else {
                assertTrue("Voltage cannot exceed 4V",voltage <= 4.0);
                assertTrue("Voltage cannot exceed -4V",voltage >= -4.0);
            }
            writer.println(currentTime+"\t"+voltage+"\t"+position+"\t"+hallEffect()+"\t"+elevator.state+"\t"+elevator.getGoal());
            SimulateTime(kDt, voltage);

        }
        assertEquals("Elevator should be running after simulation.",Elevator.State.RUNNING, elevator.state);
        assertTrue("Elevator should be at specified position.",Math.abs(position - elevator.getGoal()) < 0.01);
    }



    @After
    public void closeWriter() {
        writer.close();
    }


}
