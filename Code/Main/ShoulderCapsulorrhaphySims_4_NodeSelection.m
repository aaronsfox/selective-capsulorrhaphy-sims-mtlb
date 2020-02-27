function ShoulderCapsulorrhaphySims_4_NodeSelection

% @author: Aaron Fox
% Centre for Sport Research, Deakin University
% aaron.f@deakin.edu.au
% 
% This code serves to identify an appropriate number of nodes/appropriate
% mesh intervals to be used in the predictive simulations for each of the
% tasks.
%
% A similar approach to that presented by Lee & Umberger (2016) is
% followed, specifically the 'grid refinement' approach is used to
% repeatedly simulate tasks with greater node density and then compare the
% effect of this on the objective function value and the simulation
% outputs. Theoretically, the best node density is where the objective
% function value stops dramatically decreasing while also seeing minimal
% differences in the model outputs of interest.
%
% %%%%% TO DO: add Lee & Umberger reference...

    import org.opensim.modeling.*
    warning off

    %Set main directory
    mainDir = pwd;

    %Add supplementary code folder to path
    addpath(genpath('..\Supplementary'));

    %Load the dual expression based force plugin library
    cd('..\..\Plugin_DualEBCF\build\Release');
    opensimCommon.LoadOpenSimLibraryExact([pwd,'\osimDualEBCF.dll']);	

    %Create space to store results
    cd('..\..\..');
    mkdir('NodeSelection');
    
    %Set a variable for tasks to be simulated
    simTasks = [{'ConcentricUpwardReach105'};
        {'ConcentricForwardReach'};
        {'HairTouch'}];
    
    %Set up a variable to provide the value for the mesh intervals to Moco.
    %This function will test node densities of 25, 51, 101, 151 and 201. It
    %seems like the node number corresponds to mesh interval * 2 + 1, hence
    %the values listed here
    meshIntervals = [12,25,50,75,100,125];
    
    %% Loop through tasks
    %  NOTE: this would take a while if running as a whole
    for tt = 1:length(simTasks)
        
        %Set task name
        taskName = simTasks{tt};

        %Set up the Moco study
        study = MocoStudy();

        %Set up and add the model
        cd('ModelFiles');
        %Add geometry directory
        ModelVisualizer.addDirToGeometrySearchPaths([pwd,'\Geometry']);
        %Load model (function uses the starting 'None' model)
        osimModel = Model([pwd,'\FullShoulderModel_None.osim']);

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
                addCoordinateActuator(osimModel,char(coordSet.get(c-1).getName()),75,[inf,-inf],'_torque');
            elseif strcmp(char(coordSet.get(c-1).getName()),'pro_sup')
                %Add an idealised torque actuator.
                addCoordinateActuator(osimModel,char(coordSet.get(c-1).getName()),30,[inf,-inf],'_torque');
            elseif strcmp(char(coordSet.get(c-1).getName()),'elv_angle')
                %Add a reserve torque actuator.
                addCoordinateActuator(osimModel,char(coordSet.get(c-1).getName()),1,[inf,-inf],'_reserve');
            elseif strcmp(char(coordSet.get(c-1).getName()),'shoulder_elv') || ...
                    strcmp(char(coordSet.get(c-1).getName()),'shoulder_rot')
                %Add a reserve torque actuator.
                addCoordinateActuator(osimModel,char(coordSet.get(c-1).getName()),1,[1,-1],'_reserve');
            end
        end
        clear c
        
% % %         %Switch tendon compliance back on
% % %         for m = 0:osimModel.getMuscles().getSize()-1
% % %             %Get muscle
% % %             musc = DeGrooteFregly2016Muscle.safeDownCast(osimModel.getMuscles().get(m));
% % %             %Edit muscle properties
% % %             musc.set_ignore_tendon_compliance(false);
% % % % % %             musc.set_active_force_width_scale(1.5);
% % % % % %             musc.set_ignore_passive_fiber_force(true);
% % %         end
% % %         clear m
        
        %Finalize model connections
        osimModel.finalizeConnections();
        
% % %         %Write model to fill
% % %         osimModel.print([pwd,'\FullShoulderModel_None_CompliantTendons.osim']);
        
% % %         %Setup model processor
% % %         modelProcessor = ModelProcessor([pwd,'\FullShoulderModel_None_CompliantTendons.osim']);
% % %         modelProcessor.append(ModOpIgnorePassiveFiberForcesDGF());
% % %         modelProcessor.append(ModOpScaleActiveFiberForceCurveWidthDGF(1.5));
        
        %Initialise the problem
        problem = study.updProblem();
        
        %Set model in problem
% % %         model = modelProcessor.process();
% % %         model.initSystem();
% % %         problem.setModelProcessor(modelProcessor);
        problem.setModel(osimModel);
        
        %Set time bounds on the problem
        problem.setTimeBounds(0, [0.1,1.0]);

        % Set-up task specific parameters
        %  NOTE: A number of task parameters come from data extraction from Vidt et
        %  al. work. These can be found in the Vidt_MovementValues.mat file

        %Load the task bounds data
        cd('..\SupportingData');
        taskBounds = load('Vidt_MovementValues.mat');

        %Add state bounds relevant to task
        addTaskBounds(taskName,taskBounds,problem,osimModel);
        
% % %         %%%% add this into task bounds...
% % %         
% % %         %%%% also note - task bounding of muscles needs to be fixed as
% % %         %%%% solver errors out with tendon compliance added when muscle
% % %         %%%% activation is set to zero (likely get zero tendon force which
% % %         %%%% causes 
% % %         problem.setStateInfoPattern('/forceset/.*/normalized_tendon_force', [0, 1.5], [], []);

        %Create marker end point costs for task    
        addMarkerFinalGoals(taskName,problem,osimModel);
        
        %%%% TO DO: encompass all below in add generic goals function...
        
        %Add a MocoControlCost to the problem.
        problem.addGoal(MocoControlGoal('effort',1));

        %Add a MocoFinalTimeGoal to the problem
        problem.addGoal(MocoFinalTimeGoal('time',1));
        
% % %         %Add joint reaction minimisation goal
% % %         jrfGoal = MocoJointReactionGoal('jrfGoal',1);
% % %         jrfGoal.setJointPath('/jointset/unrothum');
% % %         jrfGoal.setLoadsFrame('parent');
% % %         jrfGoal.setExpressedInFramePath('/jointset/unrothum/rotated_scap_frame');
% % %         reactionMeasure = StdVectorString();
% % %         reactionMeasure.add('force-x'); reactionMeasure.add('force-z');
% % %         jrfGoal.setReactionMeasures(reactionMeasure);
% % %         problem.addGoal(jrfGoal);
        
% % %         %Add state goals for relevant coordinates
% % %         %Shoulder elevation
% % %         elvStateGoal = MocoSumSquaredStateGoal('elvStateGoal',0.2);
% % %         elvStateGoal.setPattern('/jointset/.*/shoulder_elv/.*');
% % %         problem.addGoal(elvStateGoal);
% % %         %Elevation plane
% % %         elvAngStateGoal = MocoSumSquaredStateGoal('elvAngStateGoal',0.2);
% % %         elvAngStateGoal.setPattern('/jointset/.*/elv_angle/.*');
% % %         problem.addGoal(elvAngStateGoal);
% % %         %Shoulder rotation
% % %         rotStateGoal = MocoSumSquaredStateGoal('rotStateGoal',0.2);
% % %         rotStateGoal.setPattern('/jointset/.*/shoulder_rot/.*');
% % %         problem.addGoal(rotStateGoal);
% % %         %Elbow flexion
% % %         flexStateGoal = MocoSumSquaredStateGoal('flexStateGoal',0.2);
% % %         flexStateGoal.setPattern('/jointset/.*/elbow_flexion/.*');
% % %         problem.addGoal(flexStateGoal);
% % %         %Pronation
% % %         proStateGoal = MocoSumSquaredStateGoal('proStateGoal',0.2);
% % %         proStateGoal.setPattern('/jointset/.*/pro_sup/.*');
% % %         problem.addGoal(proStateGoal);
        
% % %         %Add a sum of squared states goal focusing on joint coordinate
% % %         %values with the aim of making the movement efficient. 
% % %         %This might hopefully reduce some of the excessive shoulder
% % %         %rotation, elblow flexion and forearm pronation sometimes observed.
% % %         stateGoal = MocoSumSquaredStateGoal('stateGoal',1);
% % %         stateGoal.setPattern('/jointset/.*/value');
% % % % % %         %Set weights for relevant states in goal
% % % % % %         stateWeights = MocoWeightSet();
% % % % % %         stateWeights.cloneAndAppend(MocoWeight('/jointset/shoulder0/elv_angle/value',1));
% % % % % %         stateWeights.cloneAndAppend(MocoWeight('/jointset/shoulder1/shoulder_elv/value',1));
% % % % % %         stateWeights.cloneAndAppend(MocoWeight('/jointset/shoulder2/shoulder_rot/value',1));
% % % % % %         stateWeights.cloneAndAppend(MocoWeight('/jointset/elbow/elbow_flexion/value',1));
% % % % % %         stateWeights.cloneAndAppend(MocoWeight('/jointset/radioulnar/pro_sup/value',1));
% % % % % %         stateGoal.setWeightSet(stateWeights);
% % %         problem.addGoal(stateGoal);

        %Navigate to storage directory for results
        cd('..\NodeSelection');

        %% Loop through the various mesh intervals for running the solver
        for mm = 1:length(meshIntervals)

            %Configure the solver.
            solver = study.initCasADiSolver();
            solver.set_num_mesh_intervals(meshIntervals(mm));
            solver.set_verbosity(2);
            solver.set_optim_solver('ipopt');
            solver.set_optim_convergence_tolerance(1e-4);
            solver.set_optim_constraint_tolerance(1e-4);

            %Set the guess to be the previous solution iteration
            if mm > 1

                %Some solution files seem to generate nan's in the last row
                %of the slack variables which generates a Casadi error when
                %attempting to use as a guess. To resolve this, a random
                %guess can be created (which seems to set the slacks to
                %zero), and be filled with the relevant data 9states,
                %controls, multipliers) from the existing solution.

                %Grab the last solution
                mocoTraj = MocoTrajectory([taskName,'_',num2str(meshIntervals(mm-1)*2+1),'nodes_solution.sto']);
% % %                 mocoTraj = MocoTrajectory([taskName,'_',num2str(meshIntervals(mm-1)*2+1),'nodes_jrfGoal_solution.sto']);

                %Create random guessusing the current solver
                randTraj = solver.createGuess();

                %Resample random guess to original trajectory intervals
                randTraj.resampleWithNumTimes(meshIntervals(mm-1)*2+1);

                %Set time for random guess to original trajectory
                randTraj.setTime(mocoTraj.getTime())

                %Refresh states in random guess from the moco trajectory
                randTraj.setStatesTrajectory(mocoTraj.exportToStatesTable());

                %Refresh controls from moco trajectory
                controlNames = mocoTraj.getControlNames();
                for cc = 0:controlNames.size()-1
                    randTraj.setControl(controlNames.get(cc),mocoTraj.getControlMat(controlNames.get(cc)))    
                end
                clear cc controlNames

                %Refresh multipliers from moco trajectory
                multiplierNames = mocoTraj.getMultiplierNames();
                for cc = 0:multiplierNames.size()-1
                    randTraj.setMultiplier(multiplierNames.get(cc),mocoTraj.getMultiplierMat(multiplierNames.get(cc)))    
                end
                clear cc multiplierNames

                %Set the guess to this newly created trajectory
                solver.setGuess(randTraj); clear mocoTraj

% % %             else
% % %                 
% % %                 %%%% Create a guess to avoid incompatible tendon length
% % %                 %%%% starting points...
% % %                 startingGuess = solver.createGuess();
% % %                                 
% % %                 %%% Set the normalised tendon force to start at 0.2 to
% % %                 %%% avoid issues
% % %                 
% % %                 %Get number of rows in guess
% % %                 numRows = startingGuess.getNumTimes();
% % %                 
% % %                 %Get model state names
% % %                 stateNames = osimModel.getStateVariableNames();
% % %                 
% % %                 %Loop through states and adjust guess if it is for
% % %                 %normalised tendon force (consistent 0.2 value)
% % %                 %Set other parameters too...
% % %                 for ii = 0:osimModel.getNumStateVariables()-1
% % %                    currentStateName = string(stateNames.getitem(ii));
% % %                    if contains(currentStateName,'normalized_tendon_force')
% % %                        startingGuess.setState(currentStateName,linspace(0.2,0.2,numRows));
% % %                    elseif contains(currentStateName,'activation')
% % %                        startingGuess.setState(currentStateName,linspace(0.01,0.01,numRows));
% % %                    elseif contains(currentStateName,'jointset')
% % %                        startingGuess.setState(currentStateName,linspace(0,0,numRows));
% % %                    end
% % %                    %Cleanup
% % %                    clear currentStateName
% % %                 end
% % %                 clear ii
% % %                 
% % %                 %Set controls to constant low value in starting guess
% % %                 
% % %                 %Get model state names
% % %                 for ii = 0:startingGuess.getControlNames().size()-1
% % %                     currentControlName = startingGuess.getControlNames().get(ii);
% % %                     startingGuess.setControl(currentControlName,linspace(0.01,0.01,numRows));
% % %                     %Cleanup
% % %                     clear currentControlName
% % %                 end
% % %                 clear ii
% % %                 
% % %                 %Cleanup
% % %                 clear numRows stateNames
% % %                 
% % %                 %Set starting guess on solver                
% % %                 solver.setGuess(startingGuess);

            end

            %Solve
            study.setName([taskName,'_',num2str(meshIntervals(mm)*2+1),'nodes']);
            clc
            %%%% Could be that the predict solution is being re-used?
                    %%%% it's not, but below comment holds
            %%%% Best bet is probably still to encapsulate entire problem
            %%%% in a function call
            predictSolution = study.solve();

            %Write solution to file. If the solver was successful, delete the
            %default .sto file that's created. If not, need to unseal before
            %writing

            %%%% the writing of the files can be automatically fixed by
            %%%% changing the name in the study object...

            %%%% TO DO: probably still need a check in place here to unseal
            %%%% if unsuccessful

% % %             if predictSolution.success()
% % %                 %Write the solution to file
% % %                 predictSolution.write([taskName,'_',num2str(predictSolution.getNumTimes()),'nodes_solution.sto']);
% % %                 %Delete the default solution
% % %                 delete('MocoStudy_solution.sto');
% % %             else %the solver was unsuccessful
% % %                 %First unseal the solution
% % %                 predictSolution.unseal();
% % %                 %Write to file
% % %                 predictSolution.write([taskName,'_',num2str(predictSolution.getNumTimes()),'nodes_solution.sto']);
% % %                 %Check if the default file is written and delete if so
% % %                 if isfile('MocoStudy_solution.sto')
% % %                     delete('MocoStudy_solution.sto');
% % %                 else
% % %                     %no need to delete
% % %                 end
% % %             end

        end
        clear mm

        %Return to above directory
        cd('..');
    
    end
    clear tt
    
    %% Extract and compare information from solutions for the current tasks
    
    %Navigate to results directory
    cd('NodeSelection');
    
    for tt = 1:length(simTasks)
        
        taskName = simTasks{tt};

        %Grab data from solutions
        for mm = 1:length(meshIntervals)

            %Create a nodes label for the current iteration
            nodesLabel = ['nodes_',num2str(meshIntervals(mm)*2+1)];
            if mm ~= length(meshIntervals)
                nodesLabelPlus = ['nodes_',num2str(meshIntervals(mm+1)*2+1)];
            end

            %There doesn't appear to be a way to load the solution file in as a
            %solution object, so the usual commands to get the objective value and
            %solver duration don't appear valid. Instead, we can read in the file
            %line by line and identify where these parameters are.

            %Read the current solution file line by line
            fid = fopen([taskName,'_',num2str(meshIntervals(mm)*2+1),'nodes_solution.sto']);
            tline = fgetl(fid); r = 1;
            while ischar(tline)
                C{r,1} = tline; r = r + 1;
                tline = fgetl(fid);
            end
            fclose(fid); clear fid tline
            %Identify where the objective and solver duration strings are
            objInd = find(contains(C,'objective='));
            durInd = find(contains(C,'solver_duration='));
            %Extract the values from these strings by splitting at the equals sign
            %and converting to double type
            format long
            objSplit = strsplit(C{objInd,1},'=');
            objVal.(char(taskName))(mm,1) = str2double(objSplit(2));
            durSplit = strsplit(C{durInd,1},'=');
            solDur.(char(taskName))(mm,1) = str2double(durSplit(2));
            %Cleanup
            clear objInd durInd C

            %Load the solution of interest and place data in a matlab friendly format
            currSolution = MocoTrajectory([taskName,'_',num2str(meshIntervals(mm)*2+1),'nodes_solution.sto']);

            %Extract the column headers
            %NOTE: these should be consistent across the different solutions but
            %extract for each to be dafe
            statesNames = currSolution.getStateNames();
            for ii = 0:statesNames.size()-1
                statesHeaders.(char(taskName)).(char(nodesLabel)){ii+1} = ...
                    char(statesNames.get(ii));
            end
            clear ii statesNames
            controlNames = currSolution.getControlNames();
            for ii = 0:controlNames.size()-1
                controlHeaders.(char(taskName)).(char(nodesLabel)){ii+1} = ...
                    char(controlNames.get(ii));
            end
            clear ii controlNames

            %Extract the data for the current solution
            %States data
            for ii = 1:length(statesHeaders.(char(taskName)).(char(nodesLabel)))
                statesData.(char(taskName)).(char(nodesLabel))(:,ii) = ...
                    currSolution.getStateMat(statesHeaders.(char(taskName)).(char(nodesLabel)){ii});
            end
            clear ii
            %Controls data
            for ii = 1:length(controlHeaders.(char(taskName)).(char(nodesLabel)))
                controlData.(char(taskName)).(char(nodesLabel))(:,ii) = ...
                    currSolution.getControlMat(controlHeaders.(char(taskName)).(char(nodesLabel)){ii});
            end
            clear ii
            %Time data
            timeData.(char(taskName)).(char(nodesLabel))(:,1) = ...
                currSolution.getTimeMat();

            %Compare the current solution with adjacent node solution. Only run
            %this if not on the last mesh interval
            if mm ~= length(meshIntervals)
                %Create a variable name to store data under
                varLabel = [nodesLabel,'_',nodesLabelPlus];
                %Get adjacent trajectory
                adjSolution = MocoTrajectory([taskName,'_',num2str(meshIntervals(mm+1)*2+1),'nodes_solution.sto']);
                %Calculate RMS error for specific variable types
                rmsError.(char(taskName)).(char(varLabel)).coordinates = ...
                    currSolution.compareContinuousVariablesRMSPattern(adjSolution,...
                    'states','/jointset/.*/value');
                rmsError.(char(taskName)).(char(varLabel)).speeds = ...
                    currSolution.compareContinuousVariablesRMSPattern(adjSolution,...
                    'states','/jointset/.*/speed');
                rmsError.(char(taskName)).(char(varLabel)).activations = ...
                    currSolution.compareContinuousVariablesRMSPattern(adjSolution,...
                    'states','/forceset/.*/activation');
                %Calculate RMS error for specific state variables
                for ii = 1:length(statesHeaders.(char(taskName)).(char(nodesLabel)))
                    %Get current state
                    currState = statesHeaders.(char(taskName)).(char(nodesLabel)){ii};
                    %Create name to store data under
                    if contains(currState,'value')
                        %Get the joint angle coordinate via string split
                        sp = strsplit(currState,'/');
                        variable = [sp{4},'_value'];
                    elseif contains(currState,'speed')
                        %Get the joint angle coordinate via string split
                        sp = strsplit(currState,'/');
                        variable = [sp{4},'_speed'];
                    elseif contains(currState,'activation')
                        %Get the joint angle coordinate via string split
                        sp = strsplit(currState,'/');
                        variable = [sp{3},'_activation'];
                    end
                    %Calculate and store rms error
                    rmsError.(char(taskName)).(char(varLabel)).(char(variable)) = ...
                        currSolution.compareContinuousVariablesRMSPattern(adjSolution,...
                        'states',currState);
                    %Cleanup
                    clear sp variable
                end
                clear ii
                %Cleanup
                clear varLabel adjSolution currSolution
            end

        end
        clear mm
        
        %cleanup
        clear taskName
    
    end
    clear tt
        
    %% Create plots to compare node density outputs
    
    %Create color labels for task
    %These are three RGB codes from black through to lighter greys
    taskCols = [0/255 0/255 0/255;
        128/255 128/255 128/255;
        224/255 224/255 224/255];

    %Create labels for plot legend for each task
    taskLeg = [{'Upward Reach'}; {'Forward Reach'}; {'Head Touch'}];

    %Create color map for shoulder angle lines
    lineColour = [{'#000000'};
        {'#4885ed'};
        {'#f4c20d'};
        {'#db3236'};
        {'#d56dd7'};
        {'#1e9c31'}];
% % %         {'#3be2ff'};
% % %         {'#9c661c'};
% % %         {'#8c1b7f'}];

    %Create legend for node density
    for mm = 1:length(meshIntervals)
        nodeLeg{mm,1} = [num2str(meshIntervals(mm)*2+1),' Nodes'];
    end
    clear mm

    %%% Objective function value %%%

    figure; hold on
    for tt = 1:length(simTasks)
        %Plot objective value against node density
        h = plot(objVal.(simTasks{tt}));
        %Set plot parameters
        h.Color = taskCols(tt,:); h.LineWidth = 2;
        h.Marker = 'o'; h.MarkerSize = 8;
        h.MarkerEdgeColor = taskCols(tt,:); h.MarkerFaceColor = taskCols(tt,:); 
        %Cleanup
        clear h
    end
    clear tt
    %Set formatting on axes numbers
    set(gca,'FontSize',10,'FontWeight','bold','FontName','Helvetica');
    %Set axes labels
    xlabel('Number of Nodes',...
        'FontWeight','bold','FontName','Helvetica',...
        'FontSize',12);
    ylabel('Objective Function Value',...
        'FontWeight','bold','FontName','Helvetica',...
        'FontSize',12);
    %Set framing of figure
    ax = gca; box on; ax.LineWidth = 1;
    set(gca,'Layer','top');
    %Set axes limits and labels
    ax.XLim = [0.8 6.2]; ax.YLim(1) = 0; ax.YLim(2) = ax.YLim(2)*1.1;
    ax.XTick = 1:length(meshIntervals);
    for mm = 1:length(meshIntervals)
        ax.XTickLabel{mm} = num2str(meshIntervals(mm)*2+1);
    end
    clear mm
    %Add legend
    legend(taskLeg,'Location','SouthWest'); legend boxoff
    %Save figure
    print('ObjectiveValue_NodeSelection_fig.eps','-depsc2');        %eps format
    set(gcf, 'PaperPositionMode','auto')
    saveas(gcf,'ObjectiveValue_NodeSelection_fig.png');             %low res png
    saveas(gcf,'ObjectiveValue_NodeSelection_fig.fig');             %matlab figure
    print(gcf,'ObjectiveValue_NodeSelection_fig','-dtiff','-r600'); %600 dpi tif
    %Cleanup and close
    clear h ax ans
    close all
    
    %%% Solver duration value %%%

    figure; hold on
    for tt = 1:length(simTasks)
        %Plot solver duration against node density
        %COnvert from seconds to hours
        h = plot(solDur.(simTasks{tt})/60/60);
        %Set plot parameters
        h.Color = taskCols(tt,:); h.LineWidth = 2;
        h.Marker = 'o'; h.MarkerSize = 8;
        h.MarkerEdgeColor = taskCols(tt,:); h.MarkerFaceColor = taskCols(tt,:); 
        %Cleanup
        clear h
    end
    clear tt
    %Set formatting on axes numbers
    set(gca,'FontSize',10,'FontWeight','bold','FontName','Helvetica');
    %Set axes labels
    xlabel('Number of Nodes',...
        'FontWeight','bold','FontName','Helvetica',...
        'FontSize',12);
    ylabel('Solver Duration (h)',...
        'FontWeight','bold','FontName','Helvetica',...
        'FontSize',12);
    %Set framing of figure
    ax = gca; box on; ax.LineWidth = 1;
    set(gca,'Layer','top');
    %Set axes limits and labels
    ax.XLim = [0.8 6.2]; ax.YLim(1) = 0; ax.YLim(2) = ax.YLim(2)*1.1;
    ax.XTick = 1:length(meshIntervals);
    for mm = 1:length(meshIntervals)
        ax.XTickLabel{mm} = num2str(meshIntervals(mm)*2+1);
    end
    clear mm
    %Add legend
    legend(taskLeg,'Location','NorthWest'); legend boxoff
    %Save figure
    print('SolverDuration_NodeSelection_fig.eps','-depsc2');        %eps format
    set(gcf, 'PaperPositionMode','auto')
    saveas(gcf,'SolverDuration_NodeSelection_fig.png');             %low res png
    saveas(gcf,'SolverDuration_NodeSelection_fig.fig');             %matlab figure
    print(gcf,'SolverDuration_NodeSelection_fig','-dtiff','-r600'); %600 dpi tif
    %Cleanup and close
    clear h ax ans
    close all

    %%% Performance time value %%%

    figure; hold on
    for tt = 1:length(simTasks)
        %Extract time data for each node density into a single variable
        for mm = 1:length(meshIntervals)
            nodesLabel = ['nodes_',num2str(meshIntervals(mm)*2+1)];
            perfTime.(simTasks{tt})(mm,1) = timeData.(simTasks{tt}).(char(nodesLabel))(end);
            clear nodesLabel
        end
        clear mm
        %Plot performance time against node density
        h = plot(perfTime.(simTasks{tt}));
        %Set plot parameters
        h.Color = taskCols(tt,:); h.LineWidth = 2;
        h.Marker = 'o'; h.MarkerSize = 8;
        h.MarkerEdgeColor = taskCols(tt,:); h.MarkerFaceColor = taskCols(tt,:); 
        %Cleanup
        clear h
    end
    clear tt
    %Set formatting on axes numbers
    set(gca,'FontSize',10,'FontWeight','bold','FontName','Helvetica');
    %Set axes labels
    xlabel('Number of Nodes',...
        'FontWeight','bold','FontName','Helvetica',...
        'FontSize',12);
    ylabel('Task Performance Time (s)',...
        'FontWeight','bold','FontName','Helvetica',...
        'FontSize',12);
    %Set framing of figure
    ax = gca; box on; ax.LineWidth = 1;
    set(gca,'Layer','top');
    %Set axes limits and labels
    ax.XLim = [0.8 6.2]; ax.YLim(1) = 0; ax.YLim(2) = ax.YLim(2)*1.1;
    ax.XTick = 1:length(meshIntervals);
    for mm = 1:length(meshIntervals)
        ax.XTickLabel{mm} = num2str(meshIntervals(mm)*2+1);
    end
    clear mm
    %Add legend
    legend(taskLeg,'Location','SouthWest'); legend boxoff
    %Save figure
    print('PerformanceTime_NodeSelection_fig.eps','-depsc2');        %eps format
    set(gcf, 'PaperPositionMode','auto')
    saveas(gcf,'PerformanceTime_NodeSelection_fig.png');             %low res png
    saveas(gcf,'PerformanceTime_NodeSelection_fig.fig');             %matlab figure
    print(gcf,'PerformanceTime_NodeSelection_fig','-dtiff','-r600'); %600 dpi tif
    %Cleanup and close
    clear h ax ans
    close all

    %%% RMS Error -  joint coordinates %%%

    figure; hold on
    for tt = 1:length(simTasks)
        %Extract rms data for each node density change into a single variable
        for mm = 1:length(meshIntervals)-1
            %Create label to grab data
            varLabel = ['nodes_',num2str(meshIntervals(mm)*2+1),'_','nodes_',num2str(meshIntervals(mm+1)*2+1)];
            %Grab value
            rmsVal(mm,1) = rmsError.(simTasks{tt}).(char(varLabel)).coordinates;
            %Cleanup
            clear varLabel
        end
        clear mm
        %Plot RMS error against node density changes
        h = plot(rmsVal);
        %Set plot parameters
        h.Color = taskCols(tt,:); h.LineWidth = 2;
        h.Marker = 'o'; h.MarkerSize = 8;
        h.MarkerEdgeColor = taskCols(tt,:); h.MarkerFaceColor = taskCols(tt,:); 
        %Cleanup
        clear h rmsVal
    end
    clear tt
    %Set formatting on axes numbers
    set(gca,'FontSize',10,'FontWeight','bold','FontName','Helvetica');
    %Set axes labels
    xlabel('Number of Nodes',...
        'FontWeight','bold','FontName','Helvetica',...
        'FontSize',12);
    ylabel('Root Mean Square Error',...
        'FontWeight','bold','FontName','Helvetica',...
        'FontSize',12);
    %Set framing of figure
    ax = gca; box on; ax.LineWidth = 1;
    set(gca,'Layer','top');
    %Set axes limits and labels
    ax.XLim = [0.8 5.2]; ax.YLim(1) = 0; ax.YLim(2) = ax.YLim(2)*1.1;
    ax.XTick = 1:length(meshIntervals)-1;
    for mm = 2:length(meshIntervals)
        ax.XTickLabel{mm-1} = num2str(meshIntervals(mm)*2+1);
    end
    clear mm
    %Add legend
    legend(taskLeg,'Location','NorthWest'); legend boxoff
    %Save figure
    print('RMSerror_coordinates_NodeSelection_fig.eps','-depsc2');        %eps format
    set(gcf, 'PaperPositionMode','auto')
    saveas(gcf,'RMSerror_coordinates_NodeSelection_fig.png');             %low res png
    saveas(gcf,'RMSerror_coordinates_NodeSelection_fig.fig');             %matlab figure
    print(gcf,'RMSerror_coordinates_NodeSelection_fig','-dtiff','-r600'); %600 dpi tif
    %Cleanup and close
    clear h ax ans
    close all

    %%% RMS Error -  joint speeds %%%

    figure; hold on
    for tt = 1:length(simTasks)
        %Extract rms data for each node density change into a single variable
        for mm = 1:length(meshIntervals)-1
            %Create label to grab data
            varLabel = ['nodes_',num2str(meshIntervals(mm)*2+1),'_','nodes_',num2str(meshIntervals(mm+1)*2+1)];
            %Grab value
            rmsVal(mm,1) = rmsError.(simTasks{tt}).(char(varLabel)).speeds;
            %Cleanup
            clear varLabel
        end
        clear mm
        %Plot RMS error against node density changes
        h = plot(rmsVal);
        %Set plot parameters
        h.Color = taskCols(tt,:); h.LineWidth = 2;
        h.Marker = 'o'; h.MarkerSize = 8;
        h.MarkerEdgeColor = taskCols(tt,:); h.MarkerFaceColor = taskCols(tt,:); 
        %Cleanup
        clear h rmsVal
    end
    clear tt
    %Set formatting on axes numbers
    set(gca,'FontSize',10,'FontWeight','bold','FontName','Helvetica');
    %Set axes labels
    xlabel('Number of Nodes',...
        'FontWeight','bold','FontName','Helvetica',...
        'FontSize',12);
    ylabel('Root Mean Square Error',...
        'FontWeight','bold','FontName','Helvetica',...
        'FontSize',12);
    %Set framing of figure
    ax = gca; box on; ax.LineWidth = 1;
    set(gca,'Layer','top');
    %Set axes limits and labels
    ax.XLim = [0.8 5.2]; ax.YLim(1) = 0; ax.YLim(2) = ax.YLim(2)*1.1;
    ax.XTick = 1:length(meshIntervals)-1;
    for mm = 2:length(meshIntervals)
        ax.XTickLabel{mm-1} = num2str(meshIntervals(mm)*2+1);
    end
    clear mm
    %Add legend
    legend(taskLeg,'Location','NorthEast'); legend boxoff
    %Save figure
    print('RMSerror_speeds_NodeSelection_fig.eps','-depsc2');        %eps format
    set(gcf, 'PaperPositionMode','auto')
    saveas(gcf,'RMSerror_speeds_NodeSelection_fig.png');             %low res png
    saveas(gcf,'RMSerror_speeds_NodeSelection_fig.fig');             %matlab figure
    print(gcf,'RMSerror_speeds_NodeSelection_fig','-dtiff','-r600'); %600 dpi tif
    %Cleanup and close
    clear h ax ans
    close all

    %%% RMS Error -  muscle activation %%%

    figure; hold on
    for tt = 1:length(simTasks)
        %Extract rms data for each node density change into a single variable
        for mm = 1:length(meshIntervals)-1
            %Create label to grab data
            varLabel = ['nodes_',num2str(meshIntervals(mm)*2+1),'_','nodes_',num2str(meshIntervals(mm+1)*2+1)];
            %Grab value
            rmsVal(mm,1) = rmsError.(simTasks{tt}).(char(varLabel)).activations;
            %Cleanup
            clear varLabel
        end
        clear mm
        %Plot RMS error against node density changes
        h = plot(rmsVal);
        %Set plot parameters
        h.Color = taskCols(tt,:); h.LineWidth = 2;
        h.Marker = 'o'; h.MarkerSize = 8;
        h.MarkerEdgeColor = taskCols(tt,:); h.MarkerFaceColor = taskCols(tt,:); 
        %Cleanup
        clear h rmsVal
    end
    clear tt
    %Set formatting on axes numbers
    set(gca,'FontSize',10,'FontWeight','bold','FontName','Helvetica');
    %Set axes labels
    xlabel('Number of Nodes',...
        'FontWeight','bold','FontName','Helvetica',...
        'FontSize',12);
    ylabel('Root Mean Square Error',...
        'FontWeight','bold','FontName','Helvetica',...
        'FontSize',12);
    %Set framing of figure
    ax = gca; box on; ax.LineWidth = 1;
    set(gca,'Layer','top');
    %Set axes limits and labels
    ax.XLim = [0.8 5.2]; ax.YLim(1) = 0; ax.YLim(2) = ax.YLim(2)*1.1;
    ax.XTick = 1:length(meshIntervals)-1;
    for mm = 2:length(meshIntervals)
        ax.XTickLabel{mm-1} = num2str(meshIntervals(mm)*2+1);
    end
    clear mm
    %Add legend
    legend(taskLeg,'Location','NorthEast'); legend boxoff
    %Save figure
    print('RMSerror_activations_NodeSelection_fig.eps','-depsc2');        %eps format
    set(gcf, 'PaperPositionMode','auto')
    saveas(gcf,'RMSerror_activations_NodeSelection_fig.png');             %low res png
    saveas(gcf,'RMSerror_activations_NodeSelection_fig.fig');             %matlab figure
    print(gcf,'RMSerror_activations_NodeSelection_fig','-dtiff','-r600'); %600 dpi tif
    %Cleanup and close
    clear h ax ans
    close all

    %%% Shoulder joint angles %%% 

    %Loop through tasks
    for tt = 1:length(simTasks)

        figure; hold on
        %Get current figure position and triple width for the subplot
        f = gcf; figPos = f.Position;
        f.Position = [figPos(1)-figPos(4)*1.25 figPos(2) figPos(3)*3 figPos(4)];
        clear f figPos
        %Loop through mesh intervals and plot data for each shoulder angle
        for mm = 1:length(meshIntervals)
            %Create nodes label variable
            nodesLabel = ['nodes_',num2str(meshIntervals(mm)*2+1)];
            %Identify column index for each coordinate
            elvInd = find(strcmp(statesHeaders.(simTasks{tt}).(char(nodesLabel)),'/jointset/shoulder1/shoulder_elv/value'));
            angInd = find(strcmp(statesHeaders.(simTasks{tt}).(char(nodesLabel)),'/jointset/shoulder0/elv_angle/value'));
            rotInd = find(strcmp(statesHeaders.(simTasks{tt}).(char(nodesLabel)),'/jointset/shoulder2/shoulder_rot/value'));
            %Interpolate data to 0-100% of task completion
            %Grab data
            elvDat = rad2deg(statesData.(simTasks{tt}).(char(nodesLabel))(:,elvInd));
            angDat = rad2deg(statesData.(simTasks{tt}).(char(nodesLabel))(:,angInd));
            rotDat = rad2deg(statesData.(simTasks{tt}).(char(nodesLabel))(:,rotInd));
            %Create a 101 length version of the time samples
            tNorm = linspace(timeData.(simTasks{tt}).(char(nodesLabel))(1),...
                timeData.(simTasks{tt}).(char(nodesLabel))(end),101)';
            %Interpolate data
            elvDatNorm = interp1(timeData.(simTasks{tt}).(char(nodesLabel)),elvDat,tNorm);
            angDatNorm = interp1(timeData.(simTasks{tt}).(char(nodesLabel)),angDat,tNorm);
            rotDatNorm = interp1(timeData.(simTasks{tt}).(char(nodesLabel)),rotDat,tNorm);
            %Shoulder elevation
            subplot(1,3,1); hold on
            h1 = plot(0:1:100,elvDatNorm,'Color',lineColour{mm,1},'LineWidth',1.5);
            %Elevation plane
            subplot(1,3,2); hold on
            h2 = plot(0:1:100,angDatNorm,'Color',lineColour{mm,1},'LineWidth',1.5);
            %Shoulder rotation
            subplot(1,3,3); hold on
            h3 = plot(0:1:100,rotDatNorm,'Color',lineColour{mm,1},'LineWidth',1.5);
            %Cleanup
            clear h1 h2 h3 nodesLabel elvInd angInd rotInd elvDat angDat rotDat elvDatNorm angDatNorm rotDatNorm tNorm
        end
        clear mm

        %Set axes parameters
        for aa = 1:3
            subplot(1,3,aa);
            %Set font characteristics
            set(gca,'FontSize',10,'FontWeight','bold','FontName','Helvetica');
            %Set x axis label
            xlabel('0-100% Task Completion',...
                'FontWeight','bold','FontName','Helvetica',...
                'FontSize',12);
            %Set y axis label
            if aa == 1
                label = ['Shoulder Elevation (',char(176),')'];
            elseif aa == 2
                label = ['Elevation Plane (',char(176),')'];
            elseif aa == 3
                label = ['Shoulder Rotation (',char(176),')'];
            end
            ylabel(label,...
                'FontWeight','bold','FontName','Helvetica',...
                'FontSize',12);    
            clear label
            %Set axes style
            ax = gca; box on; ax.LineWidth = 1;
            set(gca,'Layer','top');
            %Set axes limits
            ax.XLim = [0 100]; ax.XTick = [0 25 50 75 100];
            clear ax
            %Place legend
            if tt == 2 && aa == 2
                legend(nodeLeg,'Location','NorthWest'); legend boxoff
            elseif tt == 3 && aa == 2
                legend(nodeLeg,'Location','NorthEast'); legend boxoff
            elseif tt == 3 && aa == 3
                legend(nodeLeg,'Location','SouthWest'); legend boxoff
            else
                legend(nodeLeg,'Location','SouthEast'); legend boxoff
            end
        end
        clear aa

        %Save figure
        print(['ShoulderAngles_',(simTasks{tt}),'_NodeSelection_fig.eps'],'-depsc2');        %eps format
        set(gcf, 'PaperPositionMode','auto')
        saveas(gcf,['ShoulderAngles_',(simTasks{tt}),'_NodeSelection_fig.png']);             %low res png
        saveas(gcf,['ShoulderAngles_',(simTasks{tt}),'_NodeSelection_fig.fig']);             %matlab figure
        print(gcf,['ShoulderAngles_',(simTasks{tt}),'_NodeSelection_fig'],'-dtiff','-r600'); %600 dpi tif
        %Close
        close all

    end
    clear tt

    %%% Shoulder joint speeds %%% 

    %Loop through tasks
    for tt = 1:length(simTasks)

        figure; hold on
        %Get current figure position and triple width for the subplot
        f = gcf; figPos = f.Position;
        f.Position = [figPos(1)-figPos(4)*1.25 figPos(2) figPos(3)*3 figPos(4)];
        clear f figPos
        %Loop through mesh intervals and plot data for each shoulder angle
        for mm = 1:length(meshIntervals)
            %Create nodes label variable
            nodesLabel = ['nodes_',num2str(meshIntervals(mm)*2+1)];
            %Identify column index for each coordinate
            elvInd = find(strcmp(statesHeaders.(simTasks{tt}).(char(nodesLabel)),'/jointset/shoulder1/shoulder_elv/speed'));
            angInd = find(strcmp(statesHeaders.(simTasks{tt}).(char(nodesLabel)),'/jointset/shoulder0/elv_angle/speed'));
            rotInd = find(strcmp(statesHeaders.(simTasks{tt}).(char(nodesLabel)),'/jointset/shoulder2/shoulder_rot/speed'));
            %Interpolate data to 0-100% of task completion
            %Grab data
            elvDat = rad2deg(statesData.(simTasks{tt}).(char(nodesLabel))(:,elvInd));
            angDat = rad2deg(statesData.(simTasks{tt}).(char(nodesLabel))(:,angInd));
            rotDat = rad2deg(statesData.(simTasks{tt}).(char(nodesLabel))(:,rotInd));
            %Create a 101 length version of the time samples
            tNorm = linspace(timeData.(simTasks{tt}).(char(nodesLabel))(1),...
                timeData.(simTasks{tt}).(char(nodesLabel))(end),101)';
            %Interpolate data
            elvDatNorm = interp1(timeData.(simTasks{tt}).(char(nodesLabel)),elvDat,tNorm);
            angDatNorm = interp1(timeData.(simTasks{tt}).(char(nodesLabel)),angDat,tNorm);
            rotDatNorm = interp1(timeData.(simTasks{tt}).(char(nodesLabel)),rotDat,tNorm);
            %Shoulder elevation
            subplot(1,3,1); hold on
            h1 = plot(0:1:100,elvDatNorm,'Color',lineColour{mm,1},'LineWidth',1.5);
            %Elevation plane
            subplot(1,3,2); hold on
            h2 = plot(0:1:100,angDatNorm,'Color',lineColour{mm,1},'LineWidth',1.5);
            %Shoulder rotation
            subplot(1,3,3); hold on
            h3 = plot(0:1:100,rotDatNorm,'Color',lineColour{mm,1},'LineWidth',1.5);
            %Cleanup
            clear h1 h2 h3 nodesLabel elvInd angInd rotInd elvDat angDat rotDat elvDatNorm angDatNorm rotDatNorm tNorm
        end
        clear mm

        %Set axes parameters
        for aa = 1:3
            subplot(1,3,aa);
            %Set font characteristics
            set(gca,'FontSize',10,'FontWeight','bold','FontName','Helvetica');
            %Set x axis label
            xlabel('0-100% Task Completion',...
                'FontWeight','bold','FontName','Helvetica',...
                'FontSize',12);
            %Set y axis label
            if aa == 1
                label = ['Shoulder Elevation Velocity (',char(176),'^{.}s^{-1})'];
            elseif aa == 2
                label = ['Elevation Plane Velocity (',char(176),'^{.}s^{-1})'];
            elseif aa == 3
                label = ['Shoulder Rotation Velocity (',char(176),'^{.}s^{-1})'];
            end
            ylabel(label,...
                'FontWeight','bold','FontName','Helvetica',...
                'FontSize',12);    
            clear label
            %Set axes style
            ax = gca; box on; ax.LineWidth = 1;
            set(gca,'Layer','top');
            %Set axes limits
            ax.XLim = [0 100]; ax.XTick = [0 25 50 75 100];
            clear ax
            %Place legend
            if tt == 1 && aa == 1
                legend(nodeLeg,'Location','SouthEast'); legend boxoff
            elseif tt == 1 && aa == 2
                legend(nodeLeg,'Location','NorthWest'); legend boxoff
            elseif tt == 2 && aa == 1
                legend(nodeLeg,'Location','SouthEast'); legend boxoff
            elseif tt == 2 && aa == 2
                legend(nodeLeg,'Location','NorthWest'); legend boxoff
            elseif tt == 3 && aa == 1
                legend(nodeLeg,'Location','SouthEast'); legend boxoff
            else
                legend(nodeLeg,'Location','NorthEast'); legend boxoff
            end
        end
        clear aa

        %Save figure
        print(['ShoulderVelocities_',(simTasks{tt}),'_NodeSelection_fig.eps'],'-depsc2');        %eps format
        set(gcf, 'PaperPositionMode','auto')
        saveas(gcf,['ShoulderVelocities_',(simTasks{tt}),'_NodeSelection_fig.png']);             %low res png
        saveas(gcf,['ShoulderVelocities_',(simTasks{tt}),'_NodeSelection_fig.fig']);             %matlab figure
        print(gcf,['ShoulderVelocities_',(simTasks{tt}),'_NodeSelection_fig'],'-dtiff','-r600'); %600 dpi tif
        %Close
        close all

    end
    clear tt

    %%% Muscle activations %%%

    %Extract a set of strings that represent the muscle activation states
    actStatesInd = find(contains(statesHeaders.(simTasks{1}).nodes_25,'/forceset'));
    actStates = statesHeaders.(simTasks{1}).nodes_25(actStatesInd);
    clear actStatesInd

    %Set a variable for positioning axes on the subplot
    subplotPos = [1:1:24,27,28];

    %Loop through tasks
    for tt = 1:length(simTasks)

        figure; hold on
        %Get current figure position and set size for the subplot
        f = gcf; figPos = f.Position;
        f.Position = [figPos(1)-figPos(4)*1.25 figPos(2)-figPos(3) figPos(3)*3 figPos(4)*2.5];
        clear f figPos
        %Loop through mesh intervals and plot data for each shoulder angle
        for mm = 1:length(meshIntervals)
            %Create nodes label variable
            nodesLabel = ['nodes_',num2str(meshIntervals(mm)*2+1)];
            %Loop through and plot each activation state on relevant subplot
            for aa = 1:length(actStates)
                %Identify column index for current state
                stateInd = find(strcmp(statesHeaders.(simTasks{tt}).(char(nodesLabel)),actStates{aa}));
                %Interpolate data to 0-100% of task completion
                %Grab data
                stateDat = statesData.(simTasks{tt}).(char(nodesLabel))(:,stateInd);
                %Create a 101 length version of the time samples
                tNorm = linspace(timeData.(simTasks{tt}).(char(nodesLabel))(1),...
                    timeData.(simTasks{tt}).(char(nodesLabel))(end),101)';
                %Interpolate data
                stateDatNorm = interp1(timeData.(simTasks{tt}).(char(nodesLabel)),stateDat,tNorm);
                %Plot data
                subplot(5,6,subplotPos(aa)); hold on
                plot(0:1:100,stateDatNorm,'Color',lineColour{mm,1},'LineWidth',1.5);
                %Cleanup
                clear stateInd stateDat tNorm stateDatNorm
            end
            clear aa
        end
        clear mm

        %Set axes parameters
        for aa = 1:length(subplotPos)
            subplot(5,6,subplotPos(aa));
            %Set font characteristics
            set(gca,'FontSize',9,'FontWeight','bold','FontName','Helvetica');
            %Set x axis label
            xlabel('0-100% Task Completion',...
                'FontWeight','bold','FontName','Helvetica',...
                'FontSize',9);
            %Set y axis label
            sp = strsplit(actStates{aa},'/');
            label = [sp{3},' Activation (0-1)'];
            ylabel(label,...
                'FontWeight','bold','FontName','Helvetica',...
                'FontSize',9);    
            clear sp label
            %Set axes style
            ax = gca; box on; ax.LineWidth = 1;
            set(gca,'Layer','top');
            %Set axes limits
            ax.XLim = [0 100]; ax.XTick = [0 50 100];
            clear ax
        end
        clear aa

        %Add legend using custom legendflex function
        ax = subplot(5,6,subplotPos(end-3));
        legendflex(ax, nodeLeg, 'anchor', {'s','n'}, 'buffer', [0 -72.5], 'nrow', length(nodeLeg));
        clear ax

        %Save figure
        print(['MuscleActivations_',(simTasks{tt}),'_NodeSelection_fig.eps'],'-depsc2');        %eps format
        set(gcf, 'PaperPositionMode','auto')
        saveas(gcf,['MuscleActivations_',(simTasks{tt}),'_NodeSelection_fig.png']);             %low res png
        saveas(gcf,['MuscleActivations_',(simTasks{tt}),'_NodeSelection_fig.fig']);             %matlab figure
        print(gcf,['MuscleActivations_',(simTasks{tt}),'_NodeSelection_fig'],'-dtiff','-r600'); %600 dpi tif
        %Close
        close all

    end
    clear tt
    

end

%----- End of ShoulderCapsulorrhaphySims_4_NodeSelection.m -----%
