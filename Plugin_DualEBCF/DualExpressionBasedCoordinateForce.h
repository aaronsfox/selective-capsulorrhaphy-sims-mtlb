#ifndef OPENSIM_DUAL_EXPRESSION_BASED_COORDINATE_FORCE_H_
#define OPENSIM_DUAL_EXPRESSION_BASED_COORDINATE_FORCE_H_
/* -------------------------------------------------------------------------- *
 *                OpenSim:  DualExpressionBasedCoordinateForce.h              *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * This plugin updates the original function of the expression based          *
 * coordinate limit force by allowing a second coordinate to be used in the   *
 * evaluated force expression (i.e. you can model force applied based on the  *
 * value of two coordinates). The force is still only applied to the first    *
 * coordinate used in the class, rather than to both. The class also has a    *
 * feature of using two expressions that can differ depending on if the first *
 * coordinate is positive (or zero) or negative.                              *
 *                                                                            *
 * This class was developed to act as 'ligaments' to restrict shoulder motion *
 * however could probably be applied to various other scenarios.              *
 *                                                                            *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Original Author(s): Nabeel Allana                                          *
 * Updated by Aaron Fox (Deakin University) in December, 2019                 *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */
// INCLUDE
#include "osimPluginDLL.h"
#include <OpenSim/Simulation/Model/Force.h>
#include <OpenSim/lepton/ExpressionProgram.h>

namespace OpenSim {

class OSIMPLUGIN_API DualExpressionBasedCoordinateForce : public Force {
OpenSim_DECLARE_CONCRETE_OBJECT(DualExpressionBasedCoordinateForce, Force);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    OpenSim_DECLARE_PROPERTY(coordinate, std::string,
        "Coordinate (name) to apply force to and use in function as q1.");
	OpenSim_DECLARE_PROPERTY(coordinateDual, std::string,
        "Coordinate (name) to use in function as q2. Does not receive a force.");
    OpenSim_DECLARE_PROPERTY(expressionPos, std::string,
        "Expression of the force magnitude as a function of the two coordinate1 (q1)"
        "and coordinate2 (q2) and their derivatives (qdot1 and qdot2)."
		"This first expression is used when coordinate1 is positive."
		"Note expression cannot have any whitespace separating characters");
	OpenSim_DECLARE_PROPERTY(expressionNeg, std::string,
        "Expression of the force magnitude as a function of the two coordinate1 (q1)"
        "and coordinate2 (q2) and their derivatives (qdot1 and qdot2)."
		"This first expression is used when coordinate1 is negative."
		"Note expression cannot have any whitespace separating characters");
//==============================================================================
// PUBLIC METHODS
//==============================================================================
    /** Default constructor. **/
    DualExpressionBasedCoordinateForce();
    /** Convenience constructor for API users.
    @param coordinate       name of the coordinate to apply the force to
	@param coordinateDual   name of the coordinate to use as second coordinate in formula
    @param expressionPos    the expression used to compute the force magnitude when coordinate is positive
	@param expressionNeg    the expression used to compute the force magnitude when coordinate is negative
    **/
    DualExpressionBasedCoordinateForce(
                const std::string& coordinate, const std::string& coordinateDual, const std::string& expressionPos, const std::string& expressionNeg);
//==============================================================================
// GET and SET parameters
//==============================================================================
    /**
    * Coordinates
    */
    void setCoordinateName(const std::string& coord) 
    {   set_coordinate(coord); }
    const std::string& getCoordinateName() const {return get_coordinate();}
	
	void setDualCoordinateName(const std::string& coordDual) 
    {   set_coordinateDual(coordDual); }
    const std::string& getCoordinateNameDual() const {return get_coordinateDual();}

    /**
    * %Set the mathematical expression that defines the force magnitude of this
    * coordinate force in terms of the coordinate value (q) and its
    * time derivative (qdot). Expressions with C-mathematical operations
    * such as +,-,*,/ and common functions: exp, pow, sqrt, sin, cos, tan, 
    * and so on are acceptable.
    * NOTE: a limitation is that the expression may not contain whitespace
    * @param expressionPos    string containing the mathematical expression that
    *                         defines the coordinate force when coordinate is positive
	* @param expressionNeg    string containing the mathematical expression that
    *                         defines the coordinate force when coordinate is negative
    */
    void setExpressionPos(const std::string& expressionPos);
	void setExpressionNeg(const std::string& expressionNeg);


    /** 
    * Get the computed Force magnitude determined by evaluating the 
    * expression above. Note, computeForce must be evaluated first,
    * and this is done automatically if the system is realized to Dynamics
    * @param state    const state (reference) for the model
    * @return         const double ref to the force magnitude
    */
    const double& getForceMagnitude(const SimTK::State& state);


//==============================================================================
// COMPUTATION
//==============================================================================
    /** Compute the coordinate force based on the user-defined expression 
        and apply it to the model */
    void computeForce(const SimTK::State& state, 
                              SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
                              SimTK::Vector& generalizedForces) const override;

    //--------------------------------------------------------------------------
    // COMPUTATIONS
    //--------------------------------------------------------------------------
    /** Force calculation operator. **/
    double calcExpressionForce( const SimTK::State& s) const;

//==============================================================================
// Reporting
//==============================================================================
    /** 
     * Provide name(s) of the quantities (column labels) of the force value(s) to be reported
     */
    OpenSim::Array<std::string> getRecordLabels() const override;
    /**
    *  Provide the value(s) to be reported that correspond to the labels
    */
    OpenSim::Array<double> getRecordValues(const SimTK::State& state) const override;

    

protected:
//==============================================================================
// ModelComponent interface
//==============================================================================
    void extendConnectToModel(Model& model) override;
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;


private:
    void setNull();
    void constructProperties();

    // parser programs for efficiently evaluating the expressions
    Lepton::ExpressionProgram _forceProgPos;
	Lepton::ExpressionProgram _forceProgNeg;
	///Lepton::ExpressionProgram _forceProg;

    // Corresponding generalized coordinate to which the force
    // is applied.
    SimTK::ReferencePtr<Coordinate> _coord;
	
	// Corresponding 2nd generalized coordinate to use in expression
    SimTK::ReferencePtr<Coordinate> _coordDual;

}; //  class DualExpressionBasedCoordinateForce

}; // namespace OpenSim

#endif // OPENSIM_DUAL_EXPRESSION_BASED_COORDINATE_FORCE_H_
