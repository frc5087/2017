package org.usfirst.frc.team5087.robot;

public class Dimensions
{
    static	final double	mm2inch = 25.4f;					// mm to an inch.

    // Inches version.
    
    static	final double	robotWidthBumperI		= 30.0f;
    static	final double	robotLengthBumperI	= 31.0f;
    
    static	final double	wheelDiameterI		= 6.0f;							// Wheel diameter.
    static	final double	wheelTreadWidthI		= 1.0f;							// Tread width.
    static	final double	wheelGapI				= 20.75f - wheelTreadWidthI;	// Distance between wheel centers.
    static	final double	wheelCircumferenceI	= wheelDiameterI * Math.PI;	// Wheel circumference.
    
    // mm Version.

    static	final double	robotWidthBumperMM	= 762.0f;
    static	final double	robotLengthBumperMM	= 787.4f;

    static	final double	wheelGapMM				= wheelGapI * mm2inch;
    static	final double	wheelCircumferenceMM	= wheelCircumferenceI * mm2inch;
    
    static final double	fieldLengthMM			= 16560.0f;
    static final double	fieldWidthhMM			= 8230.0f;

}