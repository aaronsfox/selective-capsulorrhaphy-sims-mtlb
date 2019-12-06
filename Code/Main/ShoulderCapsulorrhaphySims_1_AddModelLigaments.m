function ShoulderCapsulorrhaphySims_1_AddModelLigaments

% @author: Aaron Fox
% Centre for Sport Research, Deakin University
% aaron.f@deakin.edu.au
% 
% This code adds the necessary 'ligaments' and actuators to the basic shoulder 
% complex model for further processing.
% 
% TO DO: add further detailed notes...ligament properties based on MoBL model data...
%           Note that we left the adduction/hyperextension components are
%           left out as we don't have information on them for the later
%           optimisations
% 
% The findings of Gerber et al. highlighted the neccesity to provide restrictions
% to shoulder rotation based on the magnitude of shoulder elevation, and the 
% amoint of shoulder elevation based on the elevation plane angle. For this, a
% custom class based on the expression based coordinate force was developed
% that allowed expressions for force application to use coordinate data
% from two coordinates (i.e. the dual expression based coordinate force
% plugin). These cannot be accessed programatically through the Matlab API,
% and the initial optimisations are performed using the coordinate limit
% forces to fit these equations to, so they are added later.
% 
% TO DO: Details on plugin build...
% 
% This function should be run from it's own location (i.e. Code > Main
% directory). Without this, the manoeuvering between directories will not
% be appropriate.
% 
% TO DO: add Saul et al. and Gerber et al. references

%% Set-up

import org.opensim.modeling.*

%Set main directory
mainDir = pwd;

%% Add ligaments to model

%Navigate to model directory
cd('..\..\ModelFiles');

%Load in base model
baseModel = Model('BasicShoulderComplex.osim');

%Add coordinate actuators to model
%Coordinate actuators serve to control elevation and rotation during later
%simulations of the basic shoulder movements. Control files are provided in
%the ModelFiles directory to control these with a consistent signal.

%Create shoulder elevation coordinate actuator
elvActu = CoordinateActuator();             %constructor with default properties
elvActu.set_coordinate('shoulder_elv');     %set coordinate
elvActu.setOptimalForce(1);                 %optimal force corresponds to the force we wish to apply later
elvActu.setName('shoulder_elv_actuator');   %set actuator name

%Create shoulder rotation coordinate actuator
rotActu = CoordinateActuator();             %constructor with default properties
rotActu.set_coordinate('shoulder_rot');     %set coordinate
rotActu.setOptimalForce(0.5);               %optimal force corresponds to the force we wish to apply later
rotActu.setName('shoulder_rot_actuator');   %set actuator name

%Add forces to model
baseModel.addForce(elvActu);
baseModel.addForce(rotActu);

%Add the coordinate limit forces to the model
%These will act as 'ligaments' that restrict motion of a coordinate based
%on its own coordinate values using the coordinate limit force class (CLF)
%The parameters of the force ligaments will later be optimised to fit to
%the various capsulorrhaphy models, while the damping parameters will stay
%consisten as per Saul et al. (2015). The force vs. joint angle data from
%the optimised CLF's will subsequently be used to fit representative
%equations to that replicate these forces at the different joint angle
%combinations using the dual expression based coordinate force plugin.

%Create the shoulder elevation force CLF
elvLig = CoordinateLimitForce();             %constructor with default properties
elvLig.setName('shoulder_elv_ligaments');    %set name
elvLig.set_coordinate('shoulder_elv');       %set coordinate
elvLig.set_upper_stiffness(100);             %set upper stiffness value
elvLig.set_upper_limit(90);                  %set upper limit threshold
elvLig.set_lower_stiffness(1e20);            %set lower stiffness value
elvLig.set_lower_limit(0);                   %set lower limit threshold
elvLig.set_damping(0);                       %set damping value
elvLig.set_transition(542.84230000000002);   %set transition value

%Create the shoulder elevation damping CLF
elvDamp = CoordinateLimitForce();            %constructor with default properties
elvDamp.setName('shoulder_elv_damping');     %set name
elvDamp.set_coordinate('shoulder_elv');      %set coordinate
elvDamp.set_upper_stiffness(1e-08);          %set upper stiffness value
elvDamp.set_upper_limit(190);                %set upper limit threshold
elvDamp.set_lower_stiffness(1e-08);          %set lower stiffness value
elvDamp.set_lower_limit(190);                %set lower limit threshold
elvDamp.set_damping(0.001745);               %set damping value
elvDamp.set_transition(1);                   %set transition value

%Create the shoulder rotation force CLF
rotLig = CoordinateLimitForce();             %constructor with default properties
rotLig.setName('shoulder_rot_ligaments');    %set name
rotLig.set_coordinate('shoulder_rot');       %set coordinate
rotLig.set_upper_stiffness(100);             %set upper stiffness value
rotLig.set_upper_limit(45);                  %set upper limit threshold
rotLig.set_lower_stiffness(100);             %set lower stiffness value
rotLig.set_lower_limit(-45);                 %set lower limit threshold
rotLig.set_damping(0);                       %set damping value
rotLig.set_transition(485.46600000000001);   %set transition value

%Create the shoulder rotation damping CLF
rotDamp = CoordinateLimitForce();            %constructor with default properties
rotDamp.setName('shoulder_rot_damping');     %set name
rotDamp.set_coordinate('shoulder_rot');      %set coordinate
rotDamp.set_upper_stiffness(1e-08);          %set upper stiffness value
rotDamp.set_upper_limit(190);                %set upper limit threshold
rotDamp.set_lower_stiffness(1e-08);          %set lower stiffness value
rotDamp.set_lower_limit(190);                %set lower limit threshold
rotDamp.set_damping(0.001745);               %set damping value
rotDamp.set_transition(1);                   %set transition value

%Add forces to model
baseModel.addForce(elvLig)
baseModel.addForce(elvDamp)
baseModel.addForce(rotLig)
baseModel.addForce(rotDamp)

%% Finalise model
baseModel.finalizeConnections();
baseModel.print('BasicShoulderComplex_withForces.osim');

%%

%----- End of ShoulderCapsulorrhaphySims_1_AddModelLigaments.m -----%

end