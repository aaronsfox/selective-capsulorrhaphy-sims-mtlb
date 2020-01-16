%% Run two simulations, one with and without the constraint to:
%  (1) see if the plugin doesn't crash the simulation
%  (2) see if it makes a difference and does what it's supposed to do

%% Run baseline initial simulation

import org.opensim.modeling.*

%Set main directory
mainDir = pwd;

%Add supplementary code to path
addpath('..\..\Code\Supplementary');

%Go grab the generic shoulder model
cd('..\..\ModelFiles');
modelDir = pwd;
%Add geometry directory
ModelVisualizer.addDirToGeometrySearchPaths([modelDir,'\Geometry']);

%Load the shoulder model
osimModel = Model('FullShoulderModel.osim');
osimModel.finalizeConnections();

%Replace muscles with DeGroote et al. variant
DeGrooteFregly2016Muscle().replaceMuscles(osimModel);

%Lock the thorax joints of the model to make this a shoulder only movement
coordSet = osimModel.updCoordinateSet();
coordSet.get('thorax_tilt').set_locked(true);
coordSet.get('thorax_list').set_locked(true);
coordSet.get('thorax_rotation').set_locked(true);
coordSet.get('thorax_tx').set_locked(true);
coordSet.get('thorax_ty').set_locked(true);
coordSet.get('thorax_tz').set_locked(true);

%Add torque actuators to each degree of freedom.
%Loop through and add coordinate actuators
%Don't add anything for the thorax
for c = 1:coordSet.getSize()
    if contains(char(coordSet.get(c-1).getName()),'thorax')
        %don't add a coordinate actuator as these coordinates won't move    
    elseif strcmp(char(coordSet.get(c-1).getName()),'elbow_flexion')
        %Add an idealised torque actuator.
        newActuator = CoordinateActuator(char(coordSet.get(c-1).getName()));
        %Set the optimal force for this coordinate
        newActuator.setOptimalForce(300);
        %Set the actuator name
        newActuator.setName([char(coordSet.get(c-1).getName()),'_torque']);
        %Set min and max controls
        newActuator.setMaxControl(Inf)
        newActuator.setMinControl(-Inf)
        %Append new actuator to model
        osimModel.getForceSet().cloneAndAppend(newActuator);
    elseif strcmp(char(coordSet.get(c-1).getName()),'pro_sup')
        %Add an idealised torque actuator.
        newActuator = CoordinateActuator(char(coordSet.get(c-1).getName()));
        %Set the optimal force for this coordinate
        newActuator.setOptimalForce(100);
        %Set the actuator name
        newActuator.setName([char(coordSet.get(c-1).getName()),'_torque']);
        %Set min and max controls
        newActuator.setMaxControl(Inf)
        newActuator.setMinControl(-Inf)
        %Append new actuator to model
        osimModel.getForceSet().cloneAndAppend(newActuator);
    elseif strcmp(char(coordSet.get(c-1).getName()),'elv_angle') || ...
            strcmp(char(coordSet.get(c-1).getName()),'shoulder_elv') || ...
            strcmp(char(coordSet.get(c-1).getName()),'shoulder_rot')
        %Add a reserve torque actuator.
        newActuator = CoordinateActuator(char(coordSet.get(c-1).getName()));
        %Set the optimal force for this coordinate
        newActuator.setOptimalForce(2);
        %Set the actuator name
        newActuator.setName([char(coordSet.get(c-1).getName()),'_reserve']);
        %Set min and max controls
        newActuator.setMaxControl(Inf)
        newActuator.setMinControl(-Inf)
        %Append new actuator to model
        osimModel.getForceSet().cloneAndAppend(newActuator);
    end
end
clear c

%Turn off muscle-tendon dynamics to keep the problem simple.
%This is probably already done in the model anyway
for m = 0:osimModel.getMuscles().getSize()-1
    musc = osimModel.updMuscles().get(m);
    musc.set_ignore_tendon_compliance(true);
end
clear m

%Finalise model connections
osimModel.finalizeConnections();

%Set up a Moco study
study = MocoStudy();

%Set up the problem
problem = study.updProblem();

%Set the time bounds on the problem
problem.setTimeBounds(0,[0.5,2]);

%Set the position bounds for the shoulder coordinates
%NOTE: A number of task parameters come from data extraction from Vidt et
%al. work. These can be found in the Vidt_MovementValues.mat file
cd('..\Moco_GHJRF_Constraint\test');
taskBounds = load('Vidt_MovementValues.mat');

%Get the index row for the current tasks values in the task
%bounds structure. This test function uses the upward reach 90 task so we
%will search for that (the concetric portion of the movement)
taskName = 'ConcentricUpwardReach90';
if contains(taskName,'AxillaTouch')
    taskInd = find(strcmp(taskBounds.taskNames,'AxillaTouch'));
elseif contains(taskName,'ForwardReach')
    taskInd = find(strcmp(taskBounds.taskNames,'ForwardReach'));
elseif contains(taskName,'HairTouch')
    taskInd = find(strcmp(taskBounds.taskNames,'HairTouch'));
elseif contains(taskName,'RearTouch')
    taskInd = find(strcmp(taskBounds.taskNames,'RearTouch'));
elseif contains(taskName,'UpwardReach90')
    taskInd = find(strcmp(taskBounds.taskNames,'UpwardReach90'));
elseif contains(taskName,'UpwardReach105')
    taskInd = find(strcmp(taskBounds.taskNames,'UpwardReach105'));
end

%Allocate bounds on coordinate values

%Shoulder elevation
charValue = ['/jointset/',char(coordSet.get('shoulder_elv').getJoint().getName()),'/',char(coordSet.get('shoulder_elv').getName()),'/value'];
problem.setStateInfo(charValue,MocoBounds(deg2rad(taskBounds.shoulder_elv.min(taskInd)),deg2rad(taskBounds.shoulder_elv.max(taskInd))),...
    MocoInitialBounds(deg2rad(0)),MocoFinalBounds(deg2rad(taskBounds.shoulder_elv.con_LB(taskInd)),deg2rad(taskBounds.shoulder_elv.con_UB(taskInd))));
clear charValue

%Shoulder rotation
charValue = ['/jointset/',char(coordSet.get('shoulder_rot').getJoint().getName()),'/',char(coordSet.get('shoulder_rot').getName()),'/value'];
problem.setStateInfo(charValue,MocoBounds(deg2rad(taskBounds.shoulder_rot.min(taskInd)),deg2rad(taskBounds.shoulder_rot.max(taskInd))),...
    MocoInitialBounds(deg2rad(0)),MocoFinalBounds(deg2rad(taskBounds.shoulder_rot.con_LB(taskInd)),deg2rad(taskBounds.shoulder_rot.con_UB(taskInd))));
clear charValue 

%Elevation plane
charValue = ['/jointset/',char(coordSet.get('elv_angle').getJoint().getName()),'/',char(coordSet.get('elv_angle').getName()),'/value'];
problem.setStateInfo(charValue,MocoBounds(deg2rad(taskBounds.elv_angle.min(taskInd)),deg2rad(taskBounds.elv_angle.max(taskInd))),...
    MocoInitialBounds(deg2rad(taskBounds.elv_angle.min(taskInd)),deg2rad(taskBounds.elv_angle.max(taskInd))),...
    MocoFinalBounds(deg2rad(taskBounds.elv_angle.con_LB(taskInd)),deg2rad(taskBounds.elv_angle.con_UB(taskInd))));
clear charValue

%Elbow flexion
charValue = ['/jointset/',char(coordSet.get('elbow_flexion').getJoint().getName()),'/',char(coordSet.get('elbow_flexion').getName()),'/value'];
mn = coordSet.get('elbow_flexion').getRangeMin();
if contains(taskName,'Reach')
    %Limit elbow flexion to 90 degrees
    mx = deg2rad(90);
else
    %Leave as max attainable elbow flexion
    mx = coordSet.get('elbow_flexion').getRangeMax();
end
problem.setStateInfo(charValue,MocoBounds(mn,mx),MocoInitialBounds(0))
clear mn mx charValue

%Forearm
charValue = ['/jointset/',char(coordSet.get('pro_sup').getJoint().getName()),'/',char(coordSet.get('pro_sup').getName()),'/value'];
mn = coordSet.get('pro_sup').getRangeMin();
mx = coordSet.get('pro_sup').getRangeMax();
problem.setStateInfo(charValue,MocoBounds(mn,mx),MocoInitialBounds(0))
clear mn mx charValue 

%Set the same parameters for all model coordinate speeds, including upper
%and lower bounds; and that they should start and end at rest
problem.setStateInfoPattern('/jointset/.*/speed', [-50,50], 0, 0);

%Set all muscle activation bounds to be 0-1, and that initial values must
%be zero
problem.setStateInfoPattern('/forceset/.*/activation', [0,1], 0, []);

%Set cost functions for task. These are based on marker end point positions
%and cost related to these, based on specific points where a marker is
%required to reach for the upward reach task

%Get the desired end point of the movement. This will be at a point 15
%degrees above the shoulder at a distance of 200% of forearm length.
%(note there is no prescribed distance in the Vidt paper)

%Get the position of the shoulder joint centre. Note that the 1 corresponds
%to the humphant_offset frame. This command also transforms it to the
%ground frame.
osimModel_state = osimModel.initSystem();
SJC_ground = osimModel.getJointSet().get('shoulder0').get_frames(1).getPositionInGround(osimModel_state);

%Calculate the distance of the forearm (i.e. between the elbow and wrist
%joint centre).

%Get the position of the joint centres. Joint 1 corresponds to ulna offset
%frame for elbow and joint 0 the radius offset for the radius hand joint
EJC_ground = osimModel.getJointSet().get('elbow').get_frames(1).getPositionInGround(osimModel_state);
WJC_ground = osimModel.getJointSet().get('radius_hand_r').get_frames(0).getPositionInGround(osimModel_state);

%Calculate the distance between the joint centres
elbow = [EJC_ground.get(0),EJC_ground.get(1),EJC_ground.get(2)];
wrist = [WJC_ground.get(0),WJC_ground.get(1),WJC_ground.get(2)];
FA_length = dist_markers(elbow,wrist);
clear elbow wrist

%Calculate the position two forearm length in front of the shoulder. In
%front is represented by positive X
upwardReachPoint = [SJC_ground.get(0)+(FA_length*2),SJC_ground.get(1),SJC_ground.get(2)];

%Cleanup
clear Xdist theta Ydist

%Create a marker end point cost for the reach position. Need to use the
%markers on both sides of the wrist and the top of the hand to ensure that
%the hand is placed level and palmar side down at the end - as such, need
%to create markers end points for each of these.

%Identify the distance between the two wrist markers
osimModel_state = osimModel.initSystem();
RS = osimModel.getMarkerSet().get('RS').getLocationInGround(osimModel_state);
US = osimModel.getMarkerSet().get('US').getLocationInGround(osimModel_state);
RS = [RS.get(0),RS.get(1),RS.get(2)];
US = [US.get(0),US.get(1),US.get(2)];
wristWidth = dist_markers(RS,US);

%Add and subtract half of the wrist distance from the original marker end
%point along the Z-axis to get the proposed end points for the markers. It
%is positive Z in the ground frame for the ulna marker and negative Z for
%the radius marker
US_endLoc = Vec3(upwardReachPoint(1),upwardReachPoint(2),upwardReachPoint(3)+(wristWidth/2));
RS_endLoc = Vec3(upwardReachPoint(1),upwardReachPoint(2),upwardReachPoint(3)-(wristWidth/2));

%Measure the distance from the wrist joint centre to the wri_out marker for
%prescribing where the hand needs to go.
wri_out = osimModel.getMarkerSet().get('wri_out').getLocationInGround(osimModel_state);
wri_out = [wri_out.get(0),wri_out.get(1),wri_out.get(2)];
wrist = [WJC_ground.get(0),WJC_ground.get(1),WJC_ground.get(2)];
wristHeight = dist_markers(wri_out,wrist);

%Add the wirst height amount along the y-axis from the proposed reach point
%to get the point where the wri_out marker needs to go
W_endLoc = Vec3(upwardReachPoint(1),upwardReachPoint(2)+wristHeight,upwardReachPoint(3));

%Create the end point costs equally weighted to contribute 50% to the problem
endPointCost1 = MocoMarkerFinalGoal('RS_endPoint',5);
endPointCost1.setPointName('/markerset/RS');
endPointCost1.setReferenceLocation(RS_endLoc);
endPointCost2 = MocoMarkerFinalGoal('US_endPoint',5);
endPointCost2.setPointName('/markerset/US');
endPointCost2.setReferenceLocation(US_endLoc);
endPointCost3 = MocoMarkerFinalGoal('W_endPoint',5);
endPointCost3.setPointName('/markerset/wri_out');
endPointCost3.setReferenceLocation(W_endLoc);

%Create the control and final time goals
controlGoal = MocoControlGoal('effort',1);
timeGoal = MocoFinalTimeGoal('time',1);

%Add the goals to the problem
problem.addGoal(endPointCost1);
problem.addGoal(endPointCost2);
problem.addGoal(endPointCost3);
problem.addGoal(controlGoal);
problem.addGoal(timeGoal);

%Set model to the moco problem
problem.setModelCopy(osimModel);

%Configure the solver
solver = study.initCasADiSolver();
nMeshPoints = 25; solver.set_num_mesh_intervals(nMeshPoints);
solver.set_optim_convergence_tolerance(1e-4);
solver.set_optim_constraint_tolerance(1e-4);

%Print the Moco study to file
study.print('InitialUnconstrainedMotion.omoco');
clc

%% Solve! Write the solution to file
predictSolutionUnconstrained = study.solve();

%If optimal solution is found, write and delete the original saved
%If not, then unseal and write
if exist('MocoStudy_solution.sto', 'file') == 2
    predictSolutionUnconstrained.write('predictSolutionUnconstrained.sto');
    delete('MocoStudy_solution.sto');
else
    predictSolutionUnconstrained.unseal(); 
    predictSolutionUnconstrained.write('predictSolutionUnconstrained.sto');
end

%% Re-run the simulation with the custom path constraint added

%Load the plugin library
cd('..\build\Release');
opensimCommon.LoadOpenSimLibraryExact([pwd,'\osimMocoGHJRFConstraint.dll']);
cd('..\..\test');

%It's tricky to load own plugins programatically, so need to employ a
%sneaky way around this

%Add blank control bound constraint to the problem
pathCon = MocoControlBoundConstraint();
problem.addPathConstraint(pathCon);

%Save and load the .omoco file in XML format
study.print('tempOMOCO.omoco');
[tree,RootName,~] = xml_readOSIM('tempOMOCO.omoco');

%Copy the control bound constraint and then remove it
tree.MocoStudy.MocoProblem.MocoPhase.path_constraints.MocoGHJRFConstraint = ...
    tree.MocoStudy.MocoProblem.MocoPhase.path_constraints.MocoControlBoundConstraint;
tree.MocoStudy.MocoProblem.MocoPhase.path_constraints = ...
    rmfield(tree.MocoStudy.MocoProblem.MocoPhase.path_constraints,'MocoControlBoundConstraint');

%Write it to file
DOMnode = xml_writeOSIM('newOMOCO.omoco',tree,RootName);

%%%% TO DO: solve the issue around the re-saved .omoco file not loading in.
%%%% Solution was to just copy and paste the GHJRF constraint section into
%%%% the old 'temp' one, as this was the only one that would load in as a
%%%% study...

% % % %Load in the new study
% % % newStudy = MocoStudy('tempOMOCO.omoco');

delete tempOMOCO.omoco newOMOCO.omoco

%Configure the solver
newSolver = newStudy.initCasADiSolver();
nMeshPoints = 25; newSolver.set_num_mesh_intervals(nMeshPoints);
newSolver.set_optim_convergence_tolerance(1e-4);
newSolver.set_optim_constraint_tolerance(1e-4);
%Set the guess for this new simulation as the original solution, to
%potentially speed up the optimisation
newSolver.setGuessFile([pwd,'\predictSolutionUnconstrained.sto']);

%Print the updated study file
newStudy.print('SecondConstrainedMotion.omoco');
clc

%% Solve! Write the solution to file
predictSolutionConstrained = newStudy.solve();

%If optimal solution is found, write and delete the original saved
%If not, then unseal and write
if exist('MocoStudy_solution.sto', 'file') == 2
    predictSolutionConstrained.write('predictSolutionConstrained.sto');
    delete('MocoStudy_solution.sto');
else
    predictSolutionConstrained.unseal(); 
    predictSolutionConstrained.write('predictSolutionConstrained.sto');
end

%% For a potentially quicker solution, compare an old predicted solution 
%  to a new one

%%% do this another day




