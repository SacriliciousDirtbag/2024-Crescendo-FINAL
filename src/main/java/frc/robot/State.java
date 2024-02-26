package frc.robot;

public class State {

    // horizontal carriage state
    public enum HState {
        HOME,
        LOAD_LOW,
        LOAD_CONE,
        RAMP_CUBE,
        RAMP_CONE,
        SCORE_CUBE_LOW,
        SCORE_CONE_LOW,
        SCORE_CUBE_HIGH,
        SCORE_CONE_HIGH,
        EXTENDED
    };

    // vertical lift state
    public enum VState {
        HOME,
        LOAD_LOW,
        LOAD_CONE,
        RAMP_CUBE,  // load from
        RAMP_CONE,  // load from
        SCORE_CUBE_LOW,
        SCORE_CONE_LOW,
        SCORE_CUBE_HIGH,
        SCORE_CONE_HIGH,
    };

    // intake Pneumatic state
    public enum iState {
        STOP,
        IN,
        OUT
    };

    // Trapper/Amp Spinning State
    public enum tState {
        STOP,
        IN,
        OUT
    };

    //Feeder Spinning State
    public enum fState {
        STOP,
        IN,
        OUT
    };

    //Flywheel State
    public enum sState {
        STOP,
        IN,
        OUT
    };

    //Feeder ARM State
    public enum aState {
        INTAKE_POS,
        TRAP_POS,
        AIM_POS
    };
}
