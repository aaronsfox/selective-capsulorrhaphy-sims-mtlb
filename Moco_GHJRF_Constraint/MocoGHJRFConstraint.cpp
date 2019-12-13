/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoGHJRFConstraint.cpp                                      *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Aaron Fox (Deakin University)									  *
 * Original Author(s): Christopher Dembia                                     *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0          *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include "MocoGHJRFConstraint.h"
#include <Moco/MocoProblemInfo.h>
#include <OpenSim/Simulation/Model/Model.h>

using namespace OpenSim;

//MocoGHJRFConstraint::MocoGHJRFConstraint() {
//    constructProperties();
//}

//void MocoGHJRFConstraint::constructProperties() {
    //constructProperty_aValue();
	//constructProperty_bValue();
//}

void MocoGHJRFConstraint::initializeOnModelImpl(
        const Model& model, const MocoProblemInfo& problemInfo) const {
	
	// Get the joint name.
	// Cache the joint.
    m_joint = &model.getComponent<Joint>("/jointset/unrothum");
	
	// Set the function to calculate the forces on the rotated scapula 
	// frame of the model. This is a point where some specific frames in
	// the model are being used, hence the lack of flexibility in the class.
	// The second frame is a glenohumeral joint frame rotated to the scapula
	// coordinate system.
	// m_frame = &model.getComponent<Frame>("rotated_scap_frame");
	// m_frame = m_joint.getComponent<Frame>("rotated_scap_frame");
	m_frame = &model.getJointSet().get("unrothum").get_frames(2);
	
	// Set some info parameters that I think are required by the generic
	// path constraint class for when this is used in a 'normal' way
	// int numEqsPerControl;
	// numEqsPerControl = 1;
    // setNumEquations(numEqsPerControl);	// this won't be used

    MocoConstraintInfo info;
    std::vector<MocoBounds> bounds;
	bounds.emplace_back(0, SimTK::Infinity);
	bounds.emplace_back(-SimTK::Infinity, 0);

    info.setBounds(bounds);
    const_cast<MocoGHJRFConstraint*>(this)->setConstraintInfo(info);
}

void MocoGHJRFConstraint::calcPathConstraintErrorsImpl(
        const SimTK::State& state, SimTK::Vector& errors) const {
    
	
	getModel().realizeAcceleration(state);
	const auto& ground = getModel().getGround();
	
	// Compute the reaction loads on the parent frame.
    SimTK::SpatialVec reactionInGround;
    reactionInGround = m_joint->calcReactionOnParentExpressedInGround(state);
	
	// Re-express the reactions into the proper frame
    SimTK::Vec3 moment;
    SimTK::Vec3 force;
    moment = ground.expressVectorInAnotherFrame(
            state, reactionInGround[0], *m_frame);
    force = ground.expressVectorInAnotherFrame(
            state, reactionInGround[1], *m_frame);
			
	// Calculate the path constraint errorusing the equation from Kian et al. (2019)
	// This simply calculates the error where the combined shear and anterior 
	// components exceed the compressive component.
	// Note that the denominator (a & b) components from the equation are set manually
	// here - another aspect that limits flexibility.
	// The upper bound is set as the compressive force component
	// The lower bound is set at zero
    int iconstr = 0;
	errors[iconstr++] = (((force.get(3)*force.get(3))/(0.61*0.61)) + ((force.get(1)*force.get(1))/(0.34*0.34))) - (force.get(1)*force.get(1));
	errors[iconstr++] = (((force.get(3)*force.get(3))/(0.61*0.61)) + ((force.get(1)*force.get(1))/(0.34*0.34))) - 0;
	
}