/* -*- mode:c++ -*- ********************************************************
 * file:        BaseMobility.h
 *
 * author:      Daniel Willkomm, Andras Varga
 *
 * copyright:   (C) 2004 Telecommunication Networks Group (TKN) at
 *              Technische Universitaet Berlin, Germany.
 *
 *              (C) 2005 Andras Varga
 *
 *              This program is free software; you can redistribute it
 *              and/or modify it under the terms of the GNU General Public
 *              License as published by the Free Software Foundation; either
 *              version 2 of the License, or (at your option) any later
 *              version.
 *              For further information see file COPYING
 *              in the top level directory
 ***************************************************************************
 * part of:     framework implementation developed by tkn
 **************************************************************************/


#ifndef BASE_MOBILITY_H
#define BASE_MOBILITY_H

#include "veins/base/utils/MiXiMDefs.h"
#include "veins/base/modules/BatteryAccess.h"
#include "veins/base/utils/Coord.h"
#include "veins/base/utils/Move.h"
#include "veins/base/modules/BaseWorldUtility.h"

using Veins::BatteryAccess;

/**
 * @brief Base module for all mobility modules.
 *
 * It does not provide mobility at all, so you can use
 * it if you only want to simulate static networks.
 *
 * BaseMobility provides random placement of hosts and display
 * updates.
 * Change notifications about position changes are also posted to the
 * Blackboard.
 *
 * Another service provided by BaseMobility is border handling. If a
 * host wants to move outside the playground, this situation has to be
 * handled somehow. BaseMobility provides handling for the 4 most
 * common ways for that: reflection, wrapping, random placement and
 * raising an error. The only thing you have to do is to specify your
 * desired border handling in fixIfHostGetsOutside and call it in
 * makeMove.
 *
 * For most mobility modules the only two functions you need to
 * implement to define your own mobility module are makeMove and
 * fixIfHostGetsOutside..
 *
 * This file contains 3D implementations from Michael Swigulski.
 *
 * @ingroup mobility
 * @ingroup baseModules
 * @author Daniel Willkomm, Andras Varga
 */
class MIXIM_API BaseMobility : public BatteryAccess
{
  public:
    /**
     * @brief Selects how a node should behave if it reaches the edge
     * of the playground.
     *
     * @sa handleIfOutside()
     */
    enum BorderPolicy {
        REFLECT,       ///< reflect off the wall
        WRAP,          ///< reappear at the opposite edge (torus)
        PLACERANDOMLY, ///< placed at a randomly chosen position on the playground
        RAISEERROR     ///< stop the simulation with error
    };

    /**
     * @brief The kind field of messages
     *
     * that are used internally by this class have one of these values
     */
    enum BaseMobilityMsgKinds {
        MOVE_HOST = 21311,
        MOVE_TO_BORDER,
        /** Stores the id on which classes extending BaseMobility should
		 * continue their own kinds.*/
		LAST_BASE_MOBILITY_KIND,
    };

    /**
     * @brief Specifies which border actually has been reached
     */
    enum BorderHandling {
    	NOWHERE,   ///< not outside the playground
        X_SMALLER, ///< x smaller than 0
		X_BIGGER,  ///< x bigger or equal than playground size
		Y_SMALLER, ///< y smaller than 0
		Y_BIGGER,  ///< y bigger or equal than playground size
		Z_SMALLER, ///< z smaller than 0
		Z_BIGGER   ///< z bigger or equal than playground size
    };

  protected:

    /** @brief Pointer to BaseWorldUtility -- these two must know each other */
    BaseWorldUtility *world;

    /** @brief Stores the current position and move pattern of the host*/
    Move move;

    /** @brief Store the category of HostMove */
    const static simsignalwrap_t mobilityStateChangedSignal;

    /** @brief Time interval (in seconds) to update the hosts position*/
    simtime_t updateInterval;

    /** @brief Self message to trigger movement */
    cMessage* moveMsg;

    /** @brief debug this core module? */
    bool coreDebug;

    /** @brief Enable depth dependent scaling of nodes when 3d and tkenv is
     * used. */
    bool scaleNodeByDepth;

    /** @brief Scaling of the playground in X direction.*/
    double playgroundScaleX;
    /** @brief Scaling of the playground in Y direction.*/
    double playgroundScaleY;

    /** @brief The original width the node is displayed width.*/
    double origDisplayWidth;
    /** @brief The original height the node is displayed width.*/
    double origDisplayHeight;

    /** @brief The original size of the icon of the node.*/
    double origIconSize;
  public:

    BaseMobility();
    BaseMobility(unsigned stacksize);

    /** @brief This modules should only receive self-messages
     *
     * Dispatches border messages to handleBorderMsg() and all other
     * self-messages to handleSelfMsg()
     */
    virtual void handleMessage(cMessage *msg);

    /** @brief Initializes mobility model parameters.
     *
     * Assigns a pointer to ConnectionManager and gets a pointer to its host.
     *
     * Creates a random position for a host if the position is not given
     * as a parameter in "omnetpp.ini".
     *
     * Additionally the registration with ConnectionManager is done and it is
     * assured that the position display string tag (p) exists and contains
     * the exact (x) tag.
     *
     * If the speed of the host is bigger than 0 a first MOVE_HOST self
     * message is scheduled in stage 1
     */
    virtual void initialize(int);

    /** @brief Delete dynamically allocated objects*/
    virtual void finish(){};

    /** @brief Returns the current position at the current simulation time. */
    virtual Coord getCurrentPosition(/*simtime_t_cref stWhen = simTime()*/) const {
    	//return move.getPositionAt(stWhen);
    	return move.getStartPos();
    }

    /** @brief Returns the current speed at the current simulation time. */
    virtual Coord getCurrentSpeed() const {
    	return move.getDirection() * move.getSpeed();
    }
  protected:
    /**
     * @brief Maps the passed icon size tag (is) to an actual size in pixels.
     *
     * @param tag - the icon size tag to get the pixel size for
     * @return the size of the icon in pixels or -1 if the tag is unknown
     */
    virtual int iconSizeTagToSize(const char* tag);

    /**
	 * @brief Maps the passed size in pixels to an appropriate icon size
	 * tag (is).
	 *
	 * @param size - the icon size to get an appropriate tag for
	 * @return an icon size tag
	 */
	virtual const char* iconSizeToTag(double size);

    /** @brief Called upon arrival of a self messages
     *
     * The only self message possible is to indicate a new movement. If
     * the host is stationary this function is never called.
     *
     * every time a self message arrives makeMove is called to handle the
     * movement. Afterward updatePosition updates the position with the
     * display.
     */
    virtual void handleSelfMsg( cMessage* );

    /** @brief Called upon arrival of a border messages
     *
     * The host actually reached the border, so the startPos and startTime
     * has to be updated.
     *
     * Additionally fixIfHostGetsOutside has to be called again to catch
     * cases where the host moved in both (x and y) direction outside the
     * playground.
     */
    virtual void handleBorderMsg( cMessage* );

    /**
     * @brief Moves the host
     *
     * This function is called every time a MOVE_HOST self message
     * arrives. Here you can define the movement pattern for your
     * host.
     *
     * You should call fixIfHostGetsOutside here for border handling
     */
    virtual void makeMove(){
    	error("BaseMobility does not move the host");
    };

    /** @brief Update the position information for this node
     *
     * This function tells the Blackboard that the position has changed,
     * and it also moves the host's icon to the new position on the
     * screen.
     *
     * This function has to be called every time the position of the host
     * changes!
     */
    virtual void updatePosition();

    /** @brief Returns the width of the playground */
    double playgroundSizeX() const  {return world->getPgs()->x;}

    /** @brief Returns the height of the playground */
    double playgroundSizeY() const  {return world->getPgs()->y;}

    /** @brief Returns the height of the playground */
    double playgroundSizeZ() const  {return world->getPgs()->z;}

	/** @brief Random position somewhere in the playground. DEPRECATED: Use BaseWorldUtility::getRandomPosition() instead */
	Coord getRandomPosition() { return world->getRandomPosition();}

    /**
     * @name Border handling
     *
     * @brief Utility functions to handle hosts that move outside the
     * playground
     *
     */
    /*@{*/

    /** @brief Main border handling function
     *
	 * This function takes the BorderPolicy and all variables to be
	 * modified in case a border is reached and invokes the appropriate
	 * action. Pass dummy variables if you do not need them.
	 *
	 * The supported border policies are REFLECT, WRAP, PLACERANDOMLY, and
	 * RAISEERROR.
	 *
	 * The policy and stepTarget are mandatory parameters to
	 * pass. stepTarget is used to check whether the host actually moved
	 * outside the playground.
	 *
	 * Additional parameters to pass (in case of non atomic movements) can
	 * be targetPos (the target the host is moving to) and step (the size
	 * of a step).
	 *
	 * Angle is the direction in which the host is moving.
	 *
	 * @param policy BorderPolicy to use
	 * @param stepTarget target position of the next step of the host
	 * @param targetPos target position of the host (for non atomic movement)
	 * @param step step size of the host (for non atomic movement)
	 * @param angle direction in which the host is moving
	 *
	 * @return true if host was outside, false otherwise.
	 */
    bool handleIfOutside(BorderPolicy, Coord&, Coord&, Coord&, double&);

    /**
     * @brief Should be redefined in subclasses.
     *
     * Should invoke handleIfOutside() and pass the references to the
     * parameters to be modified.
     *
     * Additional action after border handling (such as choosing a new
     * target position if the BorderPolicy is PLACERANDOMLY) should be
     * implemented here.
     *
     * @sa HandleIfOutside
     */
    virtual void fixIfHostGetsOutside(){
    	error("fixIfHostGetsOutside has to be redefined by the user");
    };

    /**
     * @brief Checks whether the host is outside the playground and
     * returns where
     *
     * Checks whether the host moved outside and return the border it
     * crossed.
     *
     * Additionally the calculation of the step to reach the border is
     * started.
     */
    BorderHandling checkIfOutside( Coord, Coord& );

    /** @brief calculate the step to reach the border
     *
     * Calculate the step to reach the border. Additionally for the WRAP
     * policy the new start position after reaching the border is
     * calculated.
     */
    void goToBorder( BorderPolicy, BorderHandling, Coord&, Coord& );

    /**
     * @brief helperfunction for reflectIfOutside() to reflect
     * a Coordinate at a given border
     *
     * Helper function for BaseMobility::reflectIfOutside().
     *
     * Reflects a given coordinate according to the given
     * BorderHandling.
     *
     */
    void reflectCoordinate(BorderHandling border, Coord& c);

    /**
     * @brief Utility function to reflect the node if it goes outside
     * the playground.
     *
     * Reflects the host from the playground border.
     *
     * This function can update the target position, the step (for non
     * atomic movements) and the angle.
     *
     * @param wo defines the border at which to reflect
     * @param stepTarget target position of the current step of the host
     * @param targetPos target position of the host (for non atomic movements)
     * @param step step size and direction of the host (for non atomic movements)
     * @param angle direction to which the host is moving
     */
    void reflectIfOutside(BorderHandling, Coord&, Coord&, Coord&, double&);

    /**
     * @brief Utility function to wrap the node to the opposite edge
     * (torus) if it goes outside the playground.
     *
     * Wraps the host to the other playground size. Updates the target
     * position.
     *
     * @param wo defines the border at which to reflect
     * @param stepTarget target position of the current step of the host
     * @param targetPos target position of the host (for non atomic movements)
     */
    void wrapIfOutside(BorderHandling, Coord&, Coord&);

    /**
     * @brief Utility function to place the node randomly if it goes
     * outside the playground.
     *
     * Start the host at a new random position. Here the target position
     * is set to the new start position.
     *
     * You have to define a new target postion in fixIfHostGetsOutside to
     * keep the host moving.
     */
    void placeRandomlyIfOutside( Coord& );

    /*@}*/

};

#endif

