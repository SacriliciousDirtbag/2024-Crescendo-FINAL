package frc.robot;

public class State {
    
    // Intake Spinning State
    public enum iState {
        IN, 
        OUT, 
        STOP
    };

    // Trapper/Amp Spinning State
    public enum tState {
        IN,
        OUT,
        STOP
    };

    //Feeder Spinning State
    public enum fState {
        IN,
        OUT,
        STOP
    };

    //Flywheel Spinning State
    public enum sState {
        IN,
        OUT,
        STOP
    };

    //Feeder ARM State
    public enum aState {
        HOME,
        INTAKE_POS,
        TRAP_POS,
        AIM_FAR,
        AIM_NEAR
    };

    //Trap ARM State
    public enum eState {
        HOME,
        TRAP_POS,
        AIM_POS
    };
}
