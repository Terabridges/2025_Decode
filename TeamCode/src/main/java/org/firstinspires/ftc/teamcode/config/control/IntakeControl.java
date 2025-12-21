package org.firstinspires.ftc.teamcode.config.control;
import org.firstinspires.ftc.teamcode.config.subsystems.Intake;
import org.firstinspires.ftc.teamcode.config.utility.EdgeDetector;

public class IntakeControl implements Control {

    //---------------- Software ----------------
    Intake intake;
    GamepadView gp1;
    GamepadView gp2;
    EdgeDetector IntakeInRE = new EdgeDetector( () -> intake.spinnerIn());
    EdgeDetector IntakeInFE = new EdgeDetector( () -> intake.spinnerZero(), true);
    EdgeDetector IntakeOutRE = new EdgeDetector( () -> intake.spinnerOut());
    EdgeDetector IntakeOutFE = new EdgeDetector( () -> intake.spinnerZero(), true);


    //---------------- Constructor ----------------
    public IntakeControl(Intake intake, GamepadView gp1, GamepadView gp2){
        this.intake = intake;
        this.gp1 = gp1;
        this.gp2 = gp2;
    }

    //---------------- Methods ----------------


    //---------------- Interface Methods ----------------
    @Override
    public void update(){

        IntakeInRE.update(gp1.leftBumper());
        IntakeInFE.update(gp1.leftBumper());
        IntakeOutRE.update(gp1.rightBumper());
        IntakeOutFE.update(gp1.rightBumper());

    }

    @Override
    public void addTelemetry(TelemetrySink telemetry){

    }
}
