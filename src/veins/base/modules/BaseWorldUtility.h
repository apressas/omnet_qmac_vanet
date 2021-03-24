/* -*- mode:c++ -*- ********************************************************
 * file:        BaseWorldUtility.h
 *
 * author:      Tom Parker
 *
 * copyright:   (C) 2007 Parallel and Distributed Systems Group (PDS) at
 *              Technische Universiteit Delft, The Netherlands.
 *
 *              This program is free software; you can redistribute it
 *              and/or modify it under the terms of the GNU General Public
 *              License as published by the Free Software Foundation; either
 *              version 2 of the License, or (at your option) any later
 *              version.
 *              For further information see file COPYING
 *              in the top level directory
 ***************************************************************************
 * description: basic world utility class
 *              provides world-required values
 **************************************************************************/

#ifndef BASE_WORLD_UTIL_H
#define BASE_WORLD_UTIL_H

#include "veins/base/utils/MiXiMDefs.h"
#include "veins/base/utils/Coord.h"

/**
 * @brief Provides information and utility methods for the whole simulation.
 *
 * @ingroup baseModules
 */
class MIXIM_API BaseWorldUtility : public cSimpleModule
{
protected:
	/**
	 * @brief Size of the area the nodes are in (in meters)
	 *
	 * Note: The playground is in the range [0, playground size].
	 * Meaning the upper borders (at pg-size) as well as the lower
	 * borders (at 0) are part of the playground.
	 **/
    Coord playgroundSize;

    /** @brief Should the playground be treatend as a torus?*/
    bool useTorusFlag;

    /** @brief Should the world be 2-dimensional? */
    bool use2DFlag;

    /** @brief Provides a unique number for AirFrames per simulation */
    long airFrameId;

    /** @brief Stores if members are already initialized. */
    bool isInitialized;

public:
    /** @brief Speed of light in meters per second. */
	static const double speedOfLight() {
		return 299792458.0; ///< meters per second
	}

protected:
    /**
     * @brief Initializes all members accessible by other modules.
     *
     * Called once the first time another module accesses a member or during
     * initialize stage 0 at the latest.
     */
    virtual void initializeIfNecessary();

public:
    BaseWorldUtility();

    virtual void initialize(int stage);

    /**
     * @brief Returns the playgroundSize
     *
     * Note: The playground is in the range [0, playground size].
	 * Meaning the upper borders (at pg-size) as well as the lower
	 * borders (at 0) are part of the playground.
     **/
    const Coord* getPgs(){
    	initializeIfNecessary();
        return &playgroundSize;
    };

    /** @brief Returns true if our playground represents a torus (borders are connected)*/
    bool useTorus(){
    	initializeIfNecessary();
    	return useTorusFlag;
    };

	/** @brief Random position somewhere in the playground */
	virtual Coord getRandomPosition();

    /** @brief Returns true if the world is 2-dimensional */
    bool use2D()
    {
    	initializeIfNecessary();
    	return use2DFlag;
    }

    /** @brief Returns an Id for an AirFrame, at the moment simply an incremented long-value */
    long getUniqueAirFrameId()
    {
    	initializeIfNecessary();

    	// if counter has done one complete cycle and will be set to a value it already had
    	if (airFrameId == -1){
    		// print a warning
    		EV << "WARNING: AirFrameId-Counter has done one complete cycle."
    		<< " AirFrameIds are repeating now and may not be unique anymore." << endl;
    	}

    	return airFrameId++;
    }
 };

#endif
