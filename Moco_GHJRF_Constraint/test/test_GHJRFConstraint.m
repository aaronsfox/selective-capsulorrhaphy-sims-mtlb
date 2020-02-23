%% Run two simulations, one with and without the constraint to:
%  (1) see if the plugin doesn't crash the simulation
%  (2) see if it makes a difference and does what it's supposed to do

%% Set up constraiend simulation to compare to original result

import org.opensim.modeling.*
warning off

%Set main directory
mainDir = pwd;

%Add supplementary code folder to path
addpath(genpath('..\..\Code\Supplementary'));

%Load the plugin libraries
cd('..\..\Plugin_DualEBCF\build\Release');
opensimCommon.LoadOpenSimLibraryExact([pwd,'\osimDualEBCF.dll']);	
cd('..\..\..\Moco_GHJRF_Constraint\build\Release');
opensimCommon.LoadOpenSimLibraryExact([pwd,'\osimMocoGHJRFConstraint.dll']);
cd('..\..\test');

%Go grab the generic shoulder model
cd('..\..\ModelFiles');
modelDir = pwd;
%Add geometry directory
ModelVisualizer.addDirToGeometrySearchPaths([modelDir,'\Geometry']);
osimModel = Model([pwd,'\FullShoulderModel_None.osim']);

%Set mesh interval
meshInterval = 50;

%Set task name
taskName = 'ConcentricUpwardReach105';

%Set up the Moco study
study = MocoStudy();

%Initialise the problem
problem = study.updProblem();

%Lock the thorax joints of the model to make this a shoulder only movement
coordSet = osimModel.updCoordinateSet();
coordSet.get('thorax_tilt').set_locked(true);
coordSet.get('thorax_list').set_locked(true);
coordSet.get('thorax_rotation').set_locked(true);
coordSet.get('thorax_tx').set_locked(true);
coordSet.get('thorax_ty').set_locked(true);
coordSet.get('thorax_tz').set_locked(true);

%Add a 1kg mass for the reaching tasks
if contains(taskName,'Reach')
    %Get hand mass and calculate added 1kg value
    newHandMass = osimModel.getBodySet().get('hand_r').getMass() + 1;
    %Set new hand mass
    osimModel.getBodySet().get('hand_r').setMass(newHandMass);
    %Cleanup
    clear newHandMass
end

%Add relevant torque actuators to each degree of freedom.
%Loop through and add coordinate actuators
%Don't add anything for the thorax
for c = 1:coordSet.getSize()
    if contains(char(coordSet.get(c-1).getName()),'thorax')
        %don't add a coordinate actuator as these coordinates won't move    
    elseif strcmp(char(coordSet.get(c-1).getName()),'elbow_flexion')
        %Add an idealised torque actuator.
        addCoordinateActuator(osimModel,char(coordSet.get(c-1).getName()),300,[1,-1],'_torque');
    elseif strcmp(char(coordSet.get(c-1).getName()),'pro_sup')
        %Add an idealised torque actuator.
        addCoordinateActuator(osimModel,char(coordSet.get(c-1).getName()),100,[1,-1],'_torque');
    elseif strcmp(char(coordSet.get(c-1).getName()),'elv_angle')
        %Add a reserve torque actuator.
        addCoordinateActuator(osimModel,char(coordSet.get(c-1).getName()),2,[inf,-inf],'_reserve');
    elseif strcmp(char(coordSet.get(c-1).getName()),'shoulder_elv') || ...
            strcmp(char(coordSet.get(c-1).getName()),'shoulder_rot')
        %Add a reserve torque actuator.
        addCoordinateActuator(osimModel,char(coordSet.get(c-1).getName()),2,[1,-1],'_reserve');
    end
end
clear c

%Finalize model connections
osimModel.finalizeConnections();

%Set model in problem
problem.setModel(osimModel);

%Set time bounds on the problem
problem.setTimeBounds(0, [0.1,2.0]);

% Set-up task specific parameters
%  NOTE: A number of task parameters come from data extraction from Vidt et
%  al. work. These can be found in the Vidt_MovementValues.mat file

%Load the task bounds data
cd('..\SupportingData');
taskBounds = load('Vidt_MovementValues.mat');

%Add state bounds relevant to task
addTaskBounds(taskName,taskBounds,problem,osimModel);

%Create marker end point costs for task    
addMarkerFinalGoals(taskName,problem,osimModel);

%Add a MocoControlCost to the problem.
problem.addGoal(MocoControlGoal('effort',1));

%Add a MocoFinalTimeGoal to the problem
problem.addGoal(MocoFinalTimeGoal('time',1));

%Navigate to storage directory for results
cd('..\Moco_GHJRF_Constraint\test');

%Add blank control bound constraint to the problem
pathCon = MocoControlBoundConstraint();
problem.addPathConstraint(pathCon);

%Save and load the .omoco file in XML format
study.print('tempOMOCO.omoco');
[tree,RootName,~] = xml_readOSIM('tempOMOCO.omoco');

%%%% TO DO: solve the issue around the re-saved .omoco file not loading in.
%%%% Solution was to just copy and paste the GHJRF constraint section into
%%%% the old 'temp' one, as this was the only one that would load in as a
%%%% study...maybe write as text file instead of xml import

conStudy = MocoStudy('tempOMOCO.omoco');

%Configure the solver.
solver = conStudy.initCasADiSolver();
solver.set_num_mesh_intervals(meshInterval);
solver.set_verbosity(2);
solver.set_optim_solver('ipopt');
solver.set_optim_convergence_tolerance(1e-4);
solver.set_optim_constraint_tolerance(1e-4);
%Set guess
cd('..\..\NodeSelection');
solver.setGuessFile([pwd,'\ConcentricUpwardReach105_101nodes_solution.sto']);
cd('..\Moco_GHJRF_Constraint\test');

%write to file
conStudy.print('constrainedStudy.omoco');

%Solve
conStudy.setName([taskName,'_',num2str(meshInterval*2+1),'nodes_GHJRFconstrained']);
clc            
predictSolution = conStudy.solve();


%%%%% stopped solver at 626 iterations (after 6 and a half hours) -
%%%%% objective function value had largely inflated at this point


%%%%% the joint reaction force signals are a little noisy from even
%%%%% optimised solutions (not huge, but consistent up/down fluctuations in
%%%%% the signal), which could impact this testing...








