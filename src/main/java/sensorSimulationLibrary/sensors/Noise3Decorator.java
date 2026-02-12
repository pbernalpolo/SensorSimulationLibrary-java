package sensorSimulationLibrary.sensors;


import java.util.Random;

import numericalLibrary.types.Vector3;
import sensorSimulationLibrary.trajectories3d.DifferentiableTrajectory3dOfReferenceFrame;
import sensorSimulationLibrary.util.SeedManagerForRandomNumberGenerator;



/**
 * Decorates a {@link SimulatedSensorWithVector3Output} to add an offset in its measurements.
 */
public class Noise3Decorator
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
     * Last noise {@link Vector3} injected by the sensor.
     */
    protected Vector3 noise;
    
    /**
     * {@link Random} number generators used to generate the noise.
     */
    private Random[] rnds;
    
    /**
     * Standard deviation of the noise.
     */
    private double sigma;
    
    
    
    ////////////////////////////////////////////////////////////////
    // PUBLIC CONSTRUCTORS
    ////////////////////////////////////////////////////////////////
    
    /**
     * Constructs a {@link Noise3Decorator}.
     * 
     * @param wrappedTriaxialSensor		{@link SimulatedSensorWithVector3Output} to which the noise will be added.
     * @param sm    {@link SeedManagerForRandomNumberGenerator} used to extract the seeds for the random number generators from which the noise will be extracted.
     */
    public Noise3Decorator( SimulatedSensorWithVector3Output wrappedTriaxialSensor , SeedManagerForRandomNumberGenerator sm )
    {
    	this.wrappee = wrappedTriaxialSensor;
    	this.noise = Vector3.zero();
        // Initialize random number generators with seeds from sm.
        this.rnds = new Random[3];
        for(int i=0; i<3; i++) {
            this.rnds[i] = new Random();
            this.rnds[i].setSeed( sm.nextSeed() );
        }
    }
    
    
    
    ////////////////////////////////////////////////////////////////
    // PUBLIC METHODS
    ////////////////////////////////////////////////////////////////
    
    /**
     * Sets the standard deviation of the noise.
     * 
     * @param noiseSigma  standard deviation of the noise.
     */
    public void setMeasurementSigma( double noiseSigma )
    {
        this.sigma = noiseSigma;
    }
    
    
    /**
     * {@inheritDoc}
     * <p>
     * The noise is added as:
     * v_m = v_i + n
     * where  v_i  is the measurement produced by the wrapped sensor, and  n  is the noise.
     */
    public Vector3 measure( DifferentiableTrajectory3dOfReferenceFrame trajectory , double time )
    {
    	// Get the noise to be added.
    	this.noise.setX( this.rnds[0].nextGaussian() );
    	this.noise.setY( this.rnds[1].nextGaussian() );
    	this.noise.setZ( this.rnds[2].nextGaussian() );
    	this.noise.scaleInplace( this.sigma );
    	// Add the noise to the measurement.
    	return this.wrappee.measure( trajectory , time ).addInplace( this.noise );
    }
    
}
