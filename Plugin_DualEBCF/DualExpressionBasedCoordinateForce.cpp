/* -------------------------------------------------------------------------- *
 *              OpenSim:  DualExpressionBasedCoordinateForce.cpp              *
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

//=============================================================================
// INCLUDES
//=============================================================================
#include "DualExpressionBasedCoordinateForce.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/lepton/Parser.h>
#include <OpenSim/lepton/ParsedExpression.h>

using namespace OpenSim;
using namespace std;


//_____________________________________________________________________________
//Default constructor.
DualExpressionBasedCoordinateForce::DualExpressionBasedCoordinateForce()
{
    setNull();
    constructProperties();
}
//_____________________________________________________________________________
// Convenience constructor for API users.
DualExpressionBasedCoordinateForce::DualExpressionBasedCoordinateForce(
                const string& coordinate, const string& coordinateDual, const string& expressionPos, const string& expressionNeg)
{
    setNull();
    constructProperties();

    // Set properties to the passed-in values.
    setCoordinateName(coordinate);
	setDualCoordinateName(coordinateDual);
    setExpressionPos(expressionPos);
	setExpressionNeg(expressionNeg);
}

// Set the expressions for the force function and create it's lepton program 
void DualExpressionBasedCoordinateForce::setExpressionPos(const string& expressionPos) 
{
    set_expressionPos(expressionPos);
}

void DualExpressionBasedCoordinateForce::setExpressionNeg(const string& expressionNeg) 
{
    set_expressionNeg(expressionNeg);
}

//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this force to their null values.
 */
void DualExpressionBasedCoordinateForce::setNull()
{
    setAuthors("Nabeel Allana"); 
}

//_____________________________________________________________________________
/**
 * Construct properties and initialize to their default values.
 */
void DualExpressionBasedCoordinateForce::constructProperties()
{
    constructProperty_coordinate("UNASSIGNED");
	constructProperty_coordinateDual("UNASSIGNED");
    std::string zero = "0.0";
    constructProperty_expressionPos( zero );
	constructProperty_expressionNeg( zero );
}

//=============================================================================
// Connect this force element to the rest of the model.
//=============================================================================
void DualExpressionBasedCoordinateForce::extendConnectToModel(Model& aModel)
{
    Super::extendConnectToModel(aModel);

    string errorMessage;
    const string& coordName = get_coordinate();
	const string& coordNameDual = get_coordinateDual();

    string& expressionPos = upd_expressionPos();
    expressionPos.erase(
            remove_if(expressionPos.begin(), expressionPos.end(), ::isspace), 
                      expressionPos.end() );
					  
	string& expressionNeg = upd_expressionNeg();
    expressionNeg.erase(
            remove_if(expressionNeg.begin(), expressionNeg.end(), ::isspace), 
                      expressionNeg.end() );
    
    _forceProgPos = Lepton::Parser::parse(expressionPos).optimize().createProgram();
	_forceProgNeg = Lepton::Parser::parse(expressionNeg).optimize().createProgram();

    // Look up the coordinate
    if (!_model->updCoordinateSet().contains(coordName)) {
        errorMessage = "DualExpressionBasedCoordinateForce: Invalid coordinate (" + coordName + ") specified in " + getName();
        throw (Exception(errorMessage.c_str()));
    }
	if (!_model->updCoordinateSet().contains(coordNameDual)) {
        errorMessage = "DualExpressionBasedCoordinateForce: Invalid coordinate (" + coordNameDual + ") specified in " + getName();
        throw (Exception(errorMessage.c_str()));
    }
	
    _coord = &_model->updCoordinateSet().get(coordName);
	_coordDual = &_model->updCoordinateSet().get(coordNameDual);
    
    if(getName() == "")
        setName("expressionCoordForce_"+coordName+"_"+coordNameDual);
}

//=============================================================================
// Create the underlying system component(s)
//=============================================================================
void DualExpressionBasedCoordinateForce::
    extendAddToSystem(SimTK::MultibodySystem& system) const
{
    Super::extendAddToSystem(system);    // Base class first.
    addCacheVariable<double>("force_magnitude", 0.0, SimTK::Stage::Velocity);
}

//=============================================================================
// Computing
//=============================================================================
// Compute and apply the force
void DualExpressionBasedCoordinateForce::computeForce(const SimTK::State& s, 
                              SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
                              SimTK::Vector& generalizedForces) const
{
    applyGeneralizedForce(s, *_coord, calcExpressionForce(s), generalizedForces);
}

// Compute the force
double DualExpressionBasedCoordinateForce::calcExpressionForce(const SimTK::State& s ) const
{
    using namespace SimTK;
    double q1 = _coord->getValue(s);
    double qdot1 = _coord->getSpeedValue(s);
	double q2 = _coordDual->getValue(s);
    double qdot2 = _coordDual->getSpeedValue(s);
    std::map<std::string, double> forceVars;
    forceVars["q1"] = q1;
    forceVars["qdot1"] = qdot1;
	forceVars["q2"] = q2;
    forceVars["qdot2"] = qdot2;
	if (q1 >= 0.0) {
		double forceMag = _forceProgPos.evaluate(forceVars);
		setCacheVariableValue<double>(s, "force_magnitude", forceMag);
		return forceMag;
	}
	else {
		double forceMag = _forceProgNeg.evaluate(forceVars);
		setCacheVariableValue<double>(s, "force_magnitude", forceMag);
		return forceMag;
	}    
}

// get the force magnitude that has already been computed
const double& DualExpressionBasedCoordinateForce::
    getForceMagnitude(const SimTK::State& s)
{
    return getCacheVariableValue<double>(s, "force_magnitude");
}


//=============================================================================
// Reporting
//=============================================================================
// Provide names of the quantities (column labels) of the force value(s) 
// reported.
Array<std::string> DualExpressionBasedCoordinateForce::getRecordLabels() const {
    OpenSim::Array<std::string> labels("");
    labels.append(getName());
    return labels;
}
// Provide the value(s) to be reported that correspond to the labels.
Array<double> DualExpressionBasedCoordinateForce::getRecordValues(const SimTK::State& state) const {
    OpenSim::Array<double> values(0.0, 0, 1);
    values.append(calcExpressionForce(state));
    return values;
}