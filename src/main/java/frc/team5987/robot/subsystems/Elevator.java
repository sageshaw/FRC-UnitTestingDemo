package frc.team5987.robot.subsystems;


public class Elevator{
    public enum State {DISABLED, LOWERING, RUNNING}
    public State state;

    private double goal;
    private double offset;
    private double filteredGoal;

    public static double kMaxHeight = 2.1;
    public static double kMinHeight = -0.2;

    private static double Kp = 100;
    private static double Kd = 25;
    private static double dt = 0.01;
    private static double kZeroingVelocity = 0.05;

    private double lastError;

    public Elevator() {
        state = State.DISABLED;
    }

    public double update(double encoder, boolean limitTriggered, boolean enabled) {
        double kVoltageLimit = 0.0;
        switch(state) {
            case DISABLED:
                kVoltageLimit = 0.0;
                if(enabled) {
                    state = State.LOWERING;
                    filteredGoal = encoder;
                }
                break;
            case LOWERING:
                kVoltageLimit = 4.0;
                if(!enabled) {
                    state = State.DISABLED;
                } else if (limitTriggered) {
                    offset = -encoder;
                    state = State.RUNNING;
                } else {
                    filteredGoal -= kZeroingVelocity * dt;
                }

                break;
            case RUNNING:
                kVoltageLimit = 12.0;
                filteredGoal = goal;
                if (filteredGoal > kMaxHeight) {
                    filteredGoal = kMaxHeight;
                } else if (filteredGoal < kMinHeight) {
                    filteredGoal = kMinHeight;
                }
                break;
        }

        double error = filteredGoal - (encoder + offset);
        double voltage = Kp*error + Kd*(error-lastError) / dt;
        lastError = error;

        if (Math.abs(voltage)>kVoltageLimit) {
            voltage = Math.abs(voltage)/voltage * kVoltageLimit;
        }
        return voltage;
    }

    public void setGoal(double goal) {
        this.goal = goal;
    }

    public double getGoal() {
        return goal;
    }
}
