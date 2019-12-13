/* -------------------------------------------------------------------------- *
 * OpenSim Moco: RegisterTypes_osimMocoGHJRFConstraint.cpp                    *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia                                              *
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
#include "RegisterTypes_osimMocoGHJRFConstraint.h"

using namespace OpenSim;

static osimMocoGHJRFConstraintInstantiator instantiator;

OSIMMOCOGHJRFCONSTRAINT_API void RegisterTypes_osimMocoGHJRFConstraint() {
    try {
        Object::registerType(MocoGHJRFConstraint());
    } catch (const std::exception& e) {
        std::cerr << "ERROR during osimMocoGHJRFConstraint "
                     "Object registration:\n"
                  << e.what() << std::endl;
    }
}

osimMocoGHJRFConstraintInstantiator::osimMocoGHJRFConstraintInstantiator() {
    registerDllClasses();
}

void osimMocoGHJRFConstraintInstantiator::registerDllClasses() {
    RegisterTypes_osimMocoGHJRFConstraint();
}