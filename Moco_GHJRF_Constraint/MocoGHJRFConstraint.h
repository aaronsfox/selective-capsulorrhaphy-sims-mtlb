#ifndef MOCO_MOCOGHJRFCONSTRAINT_H
#define MOCO_MOCOGHJRFCONSTRAINT_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoGHJRFConstraint.h                                        *
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

#include "osimMocoGHJRFConstraintDLL.h"
#include <Moco/osimMoco.h>
#include <Moco/MocoConstraint.h>
#include <OpenSim/Simulation/SimbodyEngine/Joint.h>


namespace OpenSim {

class MocoProblemInfo;

/// This class constrains the glenohumeral joint reaction force to be between
/// specific bounds defined by the equation (1) outlined in:
///
/// Kian et al. (2019). 'Static optimization underestimates antagonist muscle
/// activity at the glenohumeral joint: A musculoskeletal modeling study.'
/// Journal of Biomechanics, 97: 109348.
///
/// This class is specific to the model it has been designed for as it uses the
/// specific joint labels and frames etc. in the calculations, but could be
/// adapted.
class OSIMMOCOGHJRFCONSTRAINT_API MocoGHJRFConstraint : public MocoPathConstraint {
    OpenSim_DECLARE_CONCRETE_OBJECT(
            MocoGHJRFConstraint, MocoPathConstraint);

public:
//==============================================================================
// PROPERTIES
//==============================================================================
    //OpenSim_DECLARE_LIST_PROPERTY(joint_path, std::string,
            //"Constrain the joint reaction force of this joint based "
            //"on an equation of its separate components.");
			
//==============================================================================
// PUBLIC METHODS
//==============================================================================			
	/** Default constructor. **/	
    MocoGHJRFConstraint() {};
	

//==============================================================================
// GET and SET parameters
//==============================================================================	

    /// @name Joint path
    /// Set the joint path (absolute path to joint in the model)
    /// constrained by this class. This is specific to the model
	/// used with this class
    /// @{
    ///void addJointPath(std::string jointPath) {
    ///    append_joint_path(std::move(jointPath));
    ///}
    ///void setJointPath(const std::vector<std::string>& jointPath) {
    ///    updProperty_joint_path().clear();
    ///    for (const auto& path : jointPath) { append_joint_path(path); }
    ///}
    ///void clearJointPath() { updProperty_joint_path().clear(); }
    ///std::vector<std::string> getJointPath() const {
    ///    std::vector<std::string> path;
    ///    for (int i = 0; i < getProperty_joint_path().size(); ++i) {
    ///        path.push_back(get_joint_path(i));
    ///    }
    ///    return path;
    ///}
    /// @}

protected:
//==============================================================================
// ModelComponent interface
//==============================================================================
    void initializeOnModelImpl(
            const Model& model, const MocoProblemInfo&) const override;

    void calcPathConstraintErrorsImpl(
            const SimTK::State& state, SimTK::Vector& errors) const override;

private:

    //void constructProperties();
	
	// Define properties
	mutable SimTK::ReferencePtr<const Joint> m_joint;
    mutable SimTK::ReferencePtr<const Frame> m_frame;

};

} // namespace OpenSim

#endif // MOCO_MOCOGHJRFCONSTRAINT_H
