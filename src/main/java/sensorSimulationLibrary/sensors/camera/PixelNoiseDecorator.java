package sensorSimulationLibrary.sensors.camera;


import java.util.List;
import java.util.Random;

import numericalLibrary.types.Vector2;
import sensorSimulationLibrary.trajectories3d.ContinuousTrajectory3dOfReferenceFrame;
import sensorSimulationLibrary.util.SeedManagerForRandomNumberGenerator;



public class PixelNoiseDecorator
{
    ////////////////////////////////////////////////////////////////
    /// PRIVATE VARIABLES
    ////////////////////////////////////////////////////////////////
	private PlaneImagePointsCamera wrappedCamera;
	private Random[] rnds;
    private double sigmaPixel;  // the standard deviation of the pixel noise
    
    
    
    ////////////////////////////////////////////////////////////////
    /// PUBLIC CONSTRUCTORS
    ////////////////////////////////////////////////////////////////
    
    public PixelNoiseDecorator( PlaneImagePointsCamera camera , SeedManagerForRandomNumberGenerator sm )
    {
    	this.wrappedCamera = camera;
        this.rnds = new Random[2];
        for(int i=0; i<this.rnds.length; i++) {
            this.rnds[i] = new Random();
            this.rnds[i].setSeed( sm.nextSeed() );
        }
    }
    
    
    
    ////////////////////////////////////////////////////////////////
    /// PUBLIC METHODS
    ////////////////////////////////////////////////////////////////
    
    public void setMeasurementSigmaPixel( double sigmaPixel )
    {
    	this.sigmaPixel = sigmaPixel;
    }
    
    
    public List<Vector2> measure( ContinuousTrajectory3dOfReferenceFrame trf , double time )
    {
    	List<Vector2> uvList = this.wrappedCamera.measure( trf , time );
    	for( Vector2 uv : uvList ) {
    		uv.addInplace( Vector2.fromComponents(
    				this.rnds[0].nextGaussian() ,
                    this.rnds[1].nextGaussian() ).scaleInplace( this.sigmaPixel ) );
    	}
        return uvList;
    }
    
}
