package sensorSimulationLibrary.sensors;


import numericalLibrary.types.Vector3;
import sensorSimulationLibrary.trajectories3d.DifferentiableTrajectory3dOfReferenceFrame;



/**
 * Decorates a {@link TriaxialSensor} to add an offset in its measurements.
 */
public class Offset3Decorator
	implements SimulatedSensorWithVector3Output
{
    ////////////////////////////////////////////////////////////////
    // PRIVATE VARIABLES
    ////////////////////////////////////////////////////////////////
    
    /**
     * {@link SimulatedSensorWithVector3Output} whose measurements will be biased.
     */
    private SimulatedSensorWithVector3Output wrappee;
    
    /**
     * Offset to be added to the {@link TriaxialSensor} measurements.
     */
    private Vector3 b;
    
    
    
    ////////////////////////////////////////////////////////////////
    // PUBLIC CONSTRUCTORS
    ////////////////////////////////////////////////////////////////
    
    /**
     * Constructs a {@link Offset3Decorator}.
     * 
     * @param wrappedTriaxialSensor		{@link SimulatedSensorWithVector3Output} to which the offset will be added.
     */
    public Offset3Decorator( SimulatedSensorWithVector3Output wrappedTriaxialSensor )
    {
    	this.wrappee = wrappedTriaxialSensor;
    	this.b = Vector3.zero();
    }
    
    
    
    ////////////////////////////////////////////////////////////////
    // PUBLIC METHODS
    ////////////////////////////////////////////////////////////////
    
    /**
     * Sets the offset of the {@link SimulatedSensorWithVector3Output}.
     * 
     * @param offset	offset value to be set.
     */
    public void setOffset( Vector3 offset )
    {
    	this.b.setTo( offset );
    }
    
    
    /**
     * {@inheritDoc}
     * <p>
     * The offset is added as:
     * v_m = v_i + b
     * where  v_i  is the measurement produced by the wrapped sensor, and  b  is the offset.
     */
    public Vector3 measure( DifferentiableTrajectory3dOfReferenceFrame trajectory , double time )
    {
    	return this.wrappee.measure( trajectory , time ).addInplace( this.b );
    }
    
}
