package sensorSimulationLibrary.util;


import java.util.List;
import java.util.ArrayList;




public class SeedManagerForRandomNumberGenerator
{
    ////////////////////////////////////////////////////////////////
    // PRIVATE VARIABLES
    ////////////////////////////////////////////////////////////////
    private List<Integer> lus;  // list to store all used seeds
    private int nextSeedToReturn;  // if we use nextSeed(), we will return the value of seedCounter, and increase it by 1
    
    
    
    ////////////////////////////////////////////////////////////////
    // PUBLIC CONSTRUCTORS
    ////////////////////////////////////////////////////////////////
    
    public SeedManagerForRandomNumberGenerator()
    {
        this.lus = new ArrayList<Integer>();
        this.nextSeedToReturn = 0;
    }
    
    
    
    ////////////////////////////////////////////////////////////////
    // PUBLIC METHODS
    ////////////////////////////////////////////////////////////////

    public void setUsed( int seedValue )
        throws Exception
    {
        if( this.isUsed( seedValue ) ) {
            throw new Exception( "[SeedManagerForRandomNumberGenerator.setUsed] Seed already used." );
        } else {
            this.setUsedWithoutChecking( seedValue );
        }
    }
    
    
    public int nextSeed()
    {
        while( this.isUsed( this.nextSeedToReturn ) ) {
            this.nextSeedToReturn++;
        }
        this.setUsedWithoutChecking( this.nextSeedToReturn );
        return this.nextSeedToReturn;
    }
    
    
    
    ////////////////////////////////////////////////////////////////
    // PRIVATE METHODS
    ////////////////////////////////////////////////////////////////
    
    private boolean isUsed( int seedValue )
    {
        return this.lus.contains( seedValue );
    }
    
    
    private void setUsedWithoutChecking( int seedValue )
    {
        this.lus.add( seedValue );
    }

}
