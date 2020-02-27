function ShoulderCapsulorrhaphySims_6_AnalyseSimulations(taskName)

% @author: Aaron Fox
% Centre for Sport Research, Deakin University
% aaron.f@deakin.edu.au
% 
% This code extracts and analyses the results of the select task.
%
% INPUTS:
%
% taskName: string containing task to be simulated. Can be one of 
% 'ConcentricUpwardReach105'; 'ConcentricForwardReach' or 'HairTouch'
    
    %Input checks
    if nargin < 1
        %Check for task
        taskNo = input('Select desired task:\n[1] Upward Reach 105\n[2] Forward Reach\n[3] Hair Touch\nEnter number: ');
        switch taskNo
            case 1
                taskName = 'ConcentricUpwardReach105';
            case 2
                taskName = 'ConcentricForwardReach';
            case 3
                taskName = 'HairTouch';
        end        
    end

    import org.opensim.modeling.*
    warning off

    %Set main directory
    mainDir = pwd;

    %Add supplementary code folder to path
    addpath(genpath('..\Supplementary'));
    
    %Load the dual expression based force plugin library
    cd('..\..\Plugin_DualEBCF\build\Release');
    opensimCommon.LoadOpenSimLibraryExact([pwd,'\osimDualEBCF.dll']);


    %% Extract simulation results
    
    %Navigate to results path
    cd(['..\..\..\SimulationResults\',taskName]);
    resultsDir = [pwd,'\'];

    %Identify the files that contain the current task name
    f = dir(['*',taskName,'*']);
    for ff = 1:length(f)
        taskFiles{ff,1} = f(ff).name;
    end
    clear f ff
    
    %Create a list of model names
    modelName = [{'None'};
        {'Anteroinferior'};
        {'Anterosuperior'};
        {'Posteroinferior'};
        {'Posterosuperior'};
        {'TotalAnterior'};
        {'TotalInferior'};
        {'TotalPosterior'};
        {'TotalSuperior'}];
    
    %Create a path to the model for each of the model names (for use in the
    %analyses later)
    cd('..\..\ModelFiles'); modelsDir = [pwd,'\'];
    ModelVisualizer.addDirToGeometrySearchPaths([modelsDir,'Geometry']);
    for mm = 1:length(modelName)
        modelFile{mm,1} = [modelsDir,'FullShoulderModel_',modelName{mm},'.osim'];        
    end
    clear mm
    
    %Navigate back to results directory
    cd(resultsDir);
    
    %% Loop through files and extract data
    for mm = 1:length(modelName)
        
        %Grab the solution related to the current model
        fileInd = logical(contains(taskFiles,modelName{mm}));
        fileName = taskFiles(fileInd);
        currSolution = MocoTrajectory(fileName{1,1});
        
        %% Extract states data
        
        %Get states data
        statesData = currSolution.getStatesTrajectoryMat();
        
        %Get states labels
        statesLabels = currSolution.getStateNames();
        for ss = 0:statesLabels.size()-1
            statesNames{ss+1,1} = char(statesLabels.get(ss));
        end
        clear ss statesLabels
        
        %Loop through states and place them in an appropriate data structure
        for ii = 1:length(statesNames)
            
            %Get current state name
            currState = statesNames{ii,1};
            %Split the string for use in switch-case statements
            splitState = strsplit(currState,'/');
            stateID = splitState{end};
            
            %Use switch to partition data appropriately
            switch stateID
                
                case 'value'
                    
                    %State is a joint coordinate
                    
                    %Get the coordinate name from the split strings
                    %Coordinate name is 4th string
                    coordName = splitState{4};
                    
                    %Put data into structure
                    simResults.(char(taskName)).(modelName{mm}).coordinateValue.(char(coordName)) = ...
                        statesData(:,ii);
                    
                    %Cleanup
                    clear coordName
                    
                case 'speed'
                    
                    %State is a joint coordinate
                    
                    %Get the coordinate name from the split strings
                    %Coordinate name is 4th string
                    coordName = splitState{4};
                    
                    %Put data into structure
                    simResults.(char(taskName)).(modelName{mm}).coordinateSpeed.(char(coordName)) = ...
                        statesData(:,ii);
                    
                    %Cleanup
                    clear coordName
                    
                case 'activation'
                    
                    %State is a muscle activation
                    
                    %Get the muscle name from the split strings
                    %Muscle name is 3rd string
                    muscName = splitState{3};
                    
                    %Put data into structure
                    simResults.(char(taskName)).(modelName{mm}).muscleActivation.(char(muscName)) = ...
                        statesData(:,ii);
                    
                    %Cleanup
                    clear muscName
                    
            end
            
        end
        clear ii
        
        %Add time array to each structure space
        simResults.(char(taskName)).(modelName{mm}).coordinateValue.time = ...
            currSolution.getTimeMat();
        simResults.(char(taskName)).(modelName{mm}).coordinateSpeed.time = ...
            currSolution.getTimeMat();
        simResults.(char(taskName)).(modelName{mm}).muscleActivation.time = ...
            currSolution.getTimeMat();
        
        %Cleanup
        clear statesNames statesData
        
        %% Run analysis on simulation for relevant model outputs
        
        %Note that the below analysis pipeline is a little convoluted, as
        %the only way to seemingly not have Matlab crash is to generate the
        %tool, print it out, load it back in and then run it - weird, but
        %this process works...
        
        %File paths are getting too long and hence the task name can't be
        %included in the analysis files, otherwise the longer ones don't
        %print out
        
        %Run a muscle analysis on Moco output
        
        %Initialise a blank analysis tool
        AnTool = AnalyzeTool();
        
        %Provide inputs to analysis tool
        %Model
        AnTool.setModelFilename(modelFile{mm});
        %States
        AnTool.setStatesFileName(fileName{1,1});
        %Time range
        AnTool.setStartTime(currSolution.getInitialTime());
        AnTool.setFinalTime(currSolution.getFinalTime());
        %Solve for equilibrium
        AnTool.setSolveForEquilibrium(true)
        %Results directory
        AnTool.setResultsDir(pwd);
        %Tool name
        AnTool.setName(modelName{mm});
        
        %Intialise force reporter analysis set
        FrAnalysis = ForceReporter();

        %Provide inputs to force reporter analysis
        %Set name
        FrAnalysis.setName('ForceReporter');
        %Set start and end time
        FrAnalysis.setStartTime(currSolution.getInitialTime());
        FrAnalysis.setEndTime(currSolution.getFinalTime());
        %Options
        FrAnalysis.setStepInterval(1);
        FrAnalysis.setInDegrees(true);

        %Add the force reporter analysis to the analyse tool
        AnTool.getAnalysisSet().cloneAndAppend(FrAnalysis);
        
        %Initialise muscle analysis tool
        MaAnalysis = MuscleAnalysis();
        
        %Provide inputs to muscle analysis
        %Set name
        MaAnalysis.setName('MuscleAnalysis');
        %Set start and end time
        MaAnalysis.setStartTime(currSolution.getInitialTime());
        MaAnalysis.setEndTime(currSolution.getFinalTime());
        %Options
        MaAnalysis.setStepInterval(1);
        MaAnalysis.setInDegrees(true);
        
        %Add the muscle analysis to the analyse tool
        AnTool.getAnalysisSet().cloneAndAppend(MaAnalysis);

        %Print tool to file
        AnTool.print([modelName{mm},'_analysis.xml']);
        
        %Re-import tool back in
        runTool = AnalyzeTool([modelName{mm},'_analysis.xml']);
        
        %Run analysis tool
        clc; runTool.run();
        
        %Cleanup existing files
        delete([modelName{mm},'_analysis.xml']);
        
        %% Extract analysis results
        
        %Load in force data
        forceData = importdata([modelName{mm},'_ForceReporter_forces.sto']);
        
        %Get a list of muscle names on first iteration
        if mm == 1
            muscleArray = ArrayStr();
            Model(modelFile{mm}).getMuscles().getNames(muscleArray);
            for kk = 0:muscleArray.getSize()-1
                muscleNames{kk+1,1} = char(muscleArray.get(kk));
            end
            clear kk
        end
        clear muscleArray
        
        %Put time variable (first column) in muscle forces structure
        simResults.(char(taskName)).(modelName{mm}).muscleForce.time = forceData.data(:,1);
                
        %Extract the muscle force data by finding and matching up columns
        %with muscle names
        for nn = 1:length(muscleNames)
            %Find matching column header
            muscleCol = logical(strcmp(muscleNames{nn},forceData.colheaders));
            %Extract the data
            simResults.(char(taskName)).(modelName{mm}).muscleForce.(muscleNames{nn}) = forceData.data(:,muscleCol);
            %Cleanup
            clear muscleCol
        end
        clear pp
        
        %Extract 'ligament' force data
        elvLigCol = logical(strcmp('shoulder_elv_DualEBCF',forceData.colheaders));
        rotLigCol = logical(strcmp('shoulder_rot_DualEBCF',forceData.colheaders));
        simResults.(char(taskName)).(modelName{mm}).ligamentForce.time = forceData.data(:,1);
        simResults.(char(taskName)).(modelName{mm}).ligamentForce.shoulder_elv_DualEBCF = forceData.data(:,elvLigCol);
        simResults.(char(taskName)).(modelName{mm}).ligamentForce.shoulder_rot_DualEBCF = forceData.data(:,rotLigCol);
        clear elvLigCol rotLigCol
        
        %Cleanup
        clear forceData
        
        %Load in fibre length data
        lenData = importdata([modelName{mm},'_MuscleAnalysis_FiberLength.sto']);
        
        %Put time variable (first column) in fiber lengths structure
        simResults.(char(taskName)).(modelName{mm}).fiberLength.time = lenData.data(:,1);
                
        %Extract the fiber length data by finding and matching up columns
        %with muscle names
        for nn = 1:length(muscleNames)
            %Find matching column header
            muscleCol = logical(strcmp(muscleNames{nn},lenData.colheaders));
            %Extract the data
            simResults.(char(taskName)).(modelName{mm}).fiberLength.(muscleNames{nn}) = lenData.data(:,muscleCol);
            %Cleanup
            clear muscleCol
        end
        clear pp
        
        %Cleanup
        clear lenData
        
        %Load in muscle length data
        lenData = importdata([modelName{mm},'_MuscleAnalysis_Length.sto']);
        
        %Put time variable (first column) in muscle lengths structure
        simResults.(char(taskName)).(modelName{mm}).muscleLength.time = lenData.data(:,1);
                
        %Extract the muscle length data by finding and matching up columns
        %with muscle names
        for nn = 1:length(muscleNames)
            %Find matching column header
            muscleCol = logical(strcmp(muscleNames{nn},lenData.colheaders));
            %Extract the data
            simResults.(char(taskName)).(modelName{mm}).muscleLength.(muscleNames{nn}) = lenData.data(:,muscleCol);
            %Cleanup
            clear muscleCol
        end
        clear pp
        
        %Cleanup
        clear lenData
        
        %Load in fiber velocity data
        velData = importdata([modelName{mm},'_MuscleAnalysis_FiberVelocity.sto']);
        
        %Put time variable (first column) in fiber velocity structure
        simResults.(char(taskName)).(modelName{mm}).fiberVelocity.time = velData.data(:,1);
                
        %Extract the fiber velocity data by finding and matching up columns
        %with muscle names
        for nn = 1:length(muscleNames)
            %Find matching column header
            muscleCol = logical(strcmp(muscleNames{nn},velData.colheaders));
            %Extract the data
            simResults.(char(taskName)).(modelName{mm}).fiberVelocity.(muscleNames{nn}) = velData.data(:,muscleCol);
            %Cleanup
            clear muscleCol
        end
        clear pp
        
        %Cleanup
        clear velData
        
        %Load in normalised fibre length data
        lenData = importdata([modelName{mm},'_MuscleAnalysis_NormalizedFiberLength.sto']);
        
        %Put time variable (first column) in normamlised fiber lengths structure
        simResults.(char(taskName)).(modelName{mm}).fiberLengthNorm.time = lenData.data(:,1);
                
        %Extract the normalised fiber length data by finding and matching up columns
        %with muscle names
        for nn = 1:length(muscleNames)
            %Find matching column header
            muscleCol = logical(strcmp(muscleNames{nn},lenData.colheaders));
            %Extract the data
            simResults.(char(taskName)).(modelName{mm}).fiberLengthNorm.(muscleNames{nn}) = lenData.data(:,muscleCol);
            %Cleanup
            clear muscleCol
        end
        clear pp
        
        %Cleanup
        clear lenData
        
        %Load in normalised fiber velocity data
        velData = importdata([modelName{mm},'_MuscleAnalysis_NormFiberVelocity.sto']);
        
        %Put time variable (first column) in fiber velocity structure
        simResults.(char(taskName)).(modelName{mm}).fiberVelocityNorm.time = velData.data(:,1);
                
        %Extract the fiber velocity data by finding and matching up columns
        %with muscle names
        for nn = 1:length(muscleNames)
            %Find matching column header
            muscleCol = logical(strcmp(muscleNames{nn},velData.colheaders));
            %Extract the data
            simResults.(char(taskName)).(modelName{mm}).fiberVelocityNorm.(muscleNames{nn}) = velData.data(:,muscleCol);
            %Cleanup
            clear muscleCol
        end
        clear pp
        
        %Cleanup
        clear velData
        
        %Load in active force data
        forceData = importdata([modelName{mm},'_MuscleAnalysis_ActiveFiberForce.sto']);
        
        %Put time variable (first column) in muscle forces structure
        simResults.(char(taskName)).(modelName{mm}).muscleForceActive.time = forceData.data(:,1);
                
        %Extract the muscle force data by finding and matching up columns
        %with muscle names
        for nn = 1:length(muscleNames)
            %Find matching column header
            muscleCol = logical(strcmp(muscleNames{nn},forceData.colheaders));
            %Extract the data
            simResults.(char(taskName)).(modelName{mm}).muscleForceActive.(muscleNames{nn}) = forceData.data(:,muscleCol);
            %Cleanup
            clear muscleCol
        end
        clear pp
        
        %Cleanup
        clear forceData
        
        %Load in passive force data
        forceData = importdata([modelName{mm},'_MuscleAnalysis_PassiveFiberForce.sto']);
        
        %Put time variable (first column) in muscle forces structure
        simResults.(char(taskName)).(modelName{mm}).muscleForcePassive.time = forceData.data(:,1);
                
        %Extract the muscle force data by finding and matching up columns
        %with muscle names
        for nn = 1:length(muscleNames)
            %Find matching column header
            muscleCol = logical(strcmp(muscleNames{nn},forceData.colheaders));
            %Extract the data
            simResults.(char(taskName)).(modelName{mm}).muscleForcePassive.(muscleNames{nn}) = forceData.data(:,muscleCol);
            %Cleanup
            clear muscleCol
        end
        clear pp
        
        %Cleanup
        clear forceData
        
        %% Calculate muscle 'cost' for each muscle taking into account it's
        %instantaneous muscle force, and maximum force capacity considering
        %it's instantaneous normalised length and velocity. See van der
        %Krogt et al. (2012), Gait Posture, 36, 113-119 for equation. Using
        %active muscle force as the variable here to only consider the cost
        %of active contributions.
        
        %Allocate time variable to cost structure
        simResults.(char(taskName)).(modelName{mm}).muscleCost.time = ...
            simResults.(char(taskName)).(modelName{mm}).muscleForce.time;
        
        %Intialise model
        osimModel = Model(modelFile{mm});
        
        %Loop through muscles to calculate cost
        for nn = 1:length(muscleNames)           
            %Get the current muscles force velocity and force length curves
            %First get the muscle in Millard form
            dgMuscle = DeGrooteFregly2016Muscle.safeDownCast(osimModel.getMuscles().get(muscleNames{nn}));
            %Get max isometric force of current muscle
            Fmax = dgMuscle.getMaxIsometricForce();     
            for d = 1:length(simResults.(char(taskName)).(modelName{mm}).muscleCost.time)
                %Get the current active force length multiplier
                FLx = dgMuscle.calcActiveForceLengthMultiplier(simResults.(char(taskName)).(modelName{mm}).fiberLengthNorm.(muscleNames{nn})(d,1));
                %Get the current force velocity multiplier
                FVx = dgMuscle.calcForceVelocityMultiplier(simResults.(char(taskName)).(modelName{mm}).fiberVelocityNorm.(muscleNames{nn})(d,1));
                %Calculate muscle cost for the current time step
                simResults.(char(taskName)).(modelName{mm}).muscleCost.(muscleNames{nn})(d,1) = ...
                    (simResults.(char(taskName)).(modelName{mm}).muscleForceActive.(muscleNames{nn})(d,1) / (Fmax * FLx * FVx))^2;
                %Cleanup
                clear FLx FVx
            end
            clear d
            %Cleanup
            clear Fmax
            
            %Calculate total cost for each muscle by cumulatively
            %integrating over time
            int = cumtrapz(simResults.(char(taskName)).(modelName{mm}).muscleCost.time,...
                simResults.(char(taskName)).(modelName{mm}).muscleCost.(muscleNames{nn}));
            simResults.(char(taskName)).(modelName{mm}).muscleCostInt.(muscleNames{nn}) = int(end);
            clear int
            
        end
        clear nn
        
        %Sum all integrated values for total muscle cost
        %Start with a value of zero, loop through and add to it
        simResults.(char(taskName)).(modelName{mm}).muscleCostTotal.all = 0;
        for nn = 1:length(muscleNames)
            simResults.(char(taskName)).(modelName{mm}).muscleCostTotal.all = ...
                simResults.(char(taskName)).(modelName{mm}).muscleCostTotal.all + ...
                simResults.(char(taskName)).(modelName{mm}).muscleCostInt.(muscleNames{nn});            
        end
        clear nn
        
        %Create muscle group specific total muscle costs
        %Create strings of groupings to assist with this (on first iteration)
        if mm == 1
            %Create muscle groups list
            muscleGroups = [{'scapulaProtractors'};
                {'externalRotators'};
                {'scapulaRetractors'};
                {'abductors'};
                {'internalRotators'};
                {'flexors'};
                {'horizontalAdductors'}];
            %Specify muscles in groups
            muscleLists.scapulaProtractors = [{'SRA1'}; {'SRA2'}; {'SRA3'}; {'PMN'}];
            muscleLists.externalRotators = [{'TMIN'}; {'INFSP'}; {'DELT3'}];
            muscleLists.scapulaRetractors = [{'RMN'}; {'RMJ1'}; {'RMJ2'}; {'TRP1'}; {'TRP2'}; {'TRP3'}];
            muscleLists.abductors = [{'DELT1'}; {'DELT2'}; {'DELT3'}; {'SUPSP'}; {'TRP1'}; {'TRP4'}; {'SRA1'}; {'SRA2'}; {'SRA3'}];
            muscleLists.adductors = [{'LAT'}; {'PECM3'}; {'TMAJ'}; {'SUBSC'}; {'CORB'}; {'LVS'}; {'RMN'}; {'RMJ1'}; {'RMJ2'}; {'PMN'}];
            muscleLists.internalRotators = [{'SUBSC'}; {'LAT'}; {'TMAJ'}; {'PECM1'}; {'PECM2'}; {'PECM3'}];
            muscleLists.flexors = [{'PECM1'}; {'DELT1'}; {'CORB'}; {'TRP1'}; {'TRP4'}; {'SRA1'}; {'SRA2'}; {'SRA3'}];
            muscleLists.horizontalAdductors = [{'PECM1'}; {'PECM2'}; {'PECM3'}; {'DELT1'}; {'SRA1'}; {'SRA2'}; {'SRA3'}; {'PMN'}];
        end
        
        %Calculate muscle group specific total muscle costs
        for gg = 1:length(muscleGroups)
            %Start with a zero value and add to it
            simResults.(char(taskName)).(modelName{mm}).muscleCostTotal.(muscleGroups{gg}) = 0;
            %Loop through muscles in group and add to cost
            for hh = 1:length(muscleLists.(muscleGroups{gg}))
                simResults.(char(taskName)).(modelName{mm}).muscleCostTotal.(muscleGroups{gg}) = ...
                    simResults.(char(taskName)).(modelName{mm}).muscleCostTotal.(muscleGroups{gg}) + ...
                    simResults.(char(taskName)).(modelName{mm}).muscleCostInt.(muscleLists.(muscleGroups{gg}){hh});                
            end
            clear hh
        end
        clear gg        
        
        %Cleanup
        clear fileInd fileName
        
    end
    clear mm
    
    %Calculate a normalise muscle cost variable to the None model
    %i.e. None values = 100%, and others represent relative changes to this
    %In this context though, 0 means '100%' so a value of 15 indicates a
    %15% increase.
    
    %Loop through models
    for mm = 1:length(modelName)        
        %Individual muscles
        for nn = 1:length(muscleNames)
            simResults.(char(taskName)).(modelName{mm}).muscleCostIntNorm.(muscleNames{nn}) = ...
                ((simResults.(char(taskName)).(modelName{mm}).muscleCostInt.(muscleNames{nn}) / ...
                simResults.(char(taskName)).None.muscleCostInt.(muscleNames{nn})) * 100)-100;                
        end        
        %All muscles
        simResults.(char(taskName)).(modelName{mm}).muscleCostTotalNorm.all = ...
            ((simResults.(char(taskName)).(modelName{mm}).muscleCostTotal.all / ...
            simResults.(char(taskName)).None.muscleCostTotal.all) * 100)-100;        
        %Muscle groups
        for gg = 1:length(muscleGroups)
            simResults.(char(taskName)).(modelName{mm}).muscleCostTotalNorm.(muscleGroups{gg}) = ...
                ((simResults.(char(taskName)).(modelName{mm}).muscleCostTotal.(muscleGroups{gg}) / ...
                simResults.(char(taskName)).None.muscleCostTotal.(muscleGroups{gg})) * 100)-100;            
        end
        clear gg       
    end
    clear mm

    %% Plot results
    
    %%%%%% TO DO: worthwhile plotting absolute values with respect to time,
    %%%%%% but may be better to present time normalised data???
    
    %Create figures directory
    mkdir('Figures'); cd('Figures');
    
    %Set colour scheme variables for the nine conditions
    lineColour = [{'#000000'};
        {'#4885ed'};
        {'#f4c20d'};
        {'#db3236'};
        {'#d56dd7'};
        {'#1e9c31'};
        {'#3be2ff'};
        {'#9c661c'};
        {'#8c1b7f'}];
    
    %Create legend for model names
    modelLeg = [{'None'};
        {'Anteroinferior'};
        {'Anterosuperior'};
        {'Posteroinferior'};
        {'Posterosuperior'};
        {'Total Anterior'};
        {'Total Inferior'};
        {'Total Posterior'};
        {'Total Superior'}];
    
    %% Performance time
    
    %%%% TO DO: could also time normalise performance to the None model...
    
    figure; hold on
    
    %Extract performance time into relevant structure
    for mm = 1:length(modelName)
        simResults.(char(taskName)).(modelName{mm}).perfTime = ...
            simResults.(char(taskName)).(modelName{mm}).coordinateValue.time(end);
        %Store variable for plotting
        perfTime(mm,1) = simResults.(char(taskName)).(modelName{mm}).coordinateValue.time(end);
    end
    clear mm
    
    %Create bar chart for performance time
    for pp = 1:length(perfTime)
        h = bar(pp,perfTime(pp));
        set(h,'FaceColor',lineColour{pp});
        set(h,'EdgeColor',lineColour{pp});        
    end
    clear pp
    
    %Set formatting on axes numbers
    set(gca,'FontSize',10,'FontWeight','bold','FontName','Helvetica');
    ax = gca; box on; ax.LineWidth = 1;
    set(gca,'Layer','top');
    %Set axes limits and labels
    ax.XLim = [0.3 9.7]; ax.YLim(1) = 0;
    for mm = 1:length(modelName)
        ax.XTickLabel{mm} = modelLeg{mm};
    end
    ax.XTickLabelRotation = 45;
    ylabel('Task Performance Time (s)',...
        'FontWeight','bold','FontName','Helvetica',...
        'FontSize',12);

    
    %%%%%%% TO DO: need to adjust figure size to support this
    
% % %     legendflex(ax, modelLeg, 'anchor', {'s','n'}, 'buffer', [0 0], 'nrow', 3);
    
    

%%%%%%%%%%%%%%%%%%%%%%%%% below plots are fine but performance time isn't
%%%%%%%%%%%%%%%%%%%%%%%%% finished...

    
    %% Shoulder joint angles
    
    %Non-time normalised
    
    figure; hold on
    %Get current figure position and triple width for the subplot
    f = gcf; figPos = f.Position;
    f.Position = [figPos(1)-figPos(4)*1.25 figPos(2) figPos(3)*3 figPos(4)];
    clear f figPos
    
    %Loop through models and plot
    for mm = 1:length(modelName)        
        %Shoulder elevation
        subplot(1,3,1); hold on
        plot(simResults.(char(taskName)).(modelName{mm}).coordinateValue.time,...
            rad2deg(simResults.(char(taskName)).(modelName{mm}).coordinateValue.shoulder_elv),...
            'Color',lineColour{mm},'LineWidth',1.5)        
        %Elevation plane
        subplot(1,3,2); hold on
        plot(simResults.(char(taskName)).(modelName{mm}).coordinateValue.time,...
            rad2deg(simResults.(char(taskName)).(modelName{mm}).coordinateValue.elv_angle),...
            'Color',lineColour{mm},'LineWidth',1.5)        
        %Shoulder rotation
        subplot(1,3,3); hold on
        plot(simResults.(char(taskName)).(modelName{mm}).coordinateValue.time,...
            rad2deg(simResults.(char(taskName)).(modelName{mm}).coordinateValue.shoulder_rot),...
            'Color',lineColour{mm},'LineWidth',1.5)        
    end
    clear mm
    
    %Find max time. Start with first model
    maxTime = simResults.(char(taskName)).(modelName{1}).coordinateValue.time(end);
    for mm = 2:length(modelName)
        if simResults.(char(taskName)).(modelName{mm}).coordinateValue.time(end) > maxTime
            maxTime = simResults.(char(taskName)).(modelName{mm}).coordinateValue.time(end);
        end            
    end
    
    %Set axes parameters
    for aa = 1:3
        subplot(1,3,aa);
        %Set font characteristics
        set(gca,'FontSize',10,'FontWeight','bold','FontName','Helvetica');
        %Set x axis label
        xlabel('Time (s)',...
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
        ax.XLim = [0 maxTime];
        clear ax
        %Place legend
        if aa == 3
            legend(modelLeg,'Location','SouthEast'); legend boxoff
        else
            legend(modelLeg,'Location','NorthWest'); legend boxoff
        end
    end
    clear aa

    %Save figure
    print(['ShoulderAngles_',(char(taskName)),'_fig.eps'],'-depsc2');        %eps format
    set(gcf, 'PaperPositionMode','auto')
    saveas(gcf,['ShoulderAngles_',(char(taskName)),'_fig.png']);             %low res png
    saveas(gcf,['ShoulderAngles_',(char(taskName)),'_fig.fig']);             %matlab figure
    print(gcf,['ShoulderAngles_',(char(taskName)),'_fig'],'-dtiff','-r600'); %600 dpi tif
    %Close
    close all
    %Cleanup
    clear maxTime
    
    %Time normalised
    
    figure; hold on
    %Get current figure position and triple width for the subplot
    f = gcf; figPos = f.Position;
    f.Position = [figPos(1)-figPos(4)*1.25 figPos(2) figPos(3)*3 figPos(4)];
    clear f figPos
    
    %Loop through models and plot
    for mm = 1:length(modelName)  
        %Create a 101 length version of the time samples
        tNorm = linspace(simResults.(char(taskName)).(modelName{mm}).coordinateValue.time(1),...
                simResults.(char(taskName)).(modelName{mm}).coordinateValue.time(end),101)';
        %Create normalised versions of shoulder angle data
        elvDatNorm = interp1(simResults.(char(taskName)).(modelName{mm}).coordinateValue.time,...
            simResults.(char(taskName)).(modelName{mm}).coordinateValue.shoulder_elv,tNorm);
        angDatNorm = interp1(simResults.(char(taskName)).(modelName{mm}).coordinateValue.time,...
            simResults.(char(taskName)).(modelName{mm}).coordinateValue.elv_angle,tNorm);
        rotDatNorm = interp1(simResults.(char(taskName)).(modelName{mm}).coordinateValue.time,...
            simResults.(char(taskName)).(modelName{mm}).coordinateValue.shoulder_rot,tNorm);
        %Shoulder elevation        
        subplot(1,3,1); hold on
        plot(0:100,rad2deg(elvDatNorm),...
            'Color',lineColour{mm},'LineWidth',1.5)        
        %Elevation plane
        subplot(1,3,2); hold on
        plot(0:100,rad2deg(angDatNorm),...
            'Color',lineColour{mm},'LineWidth',1.5)        
        %Shoulder rotation
        subplot(1,3,3); hold on
        plot(0:100,rad2deg(rotDatNorm),...
            'Color',lineColour{mm},'LineWidth',1.5)
        %Cleanup
        clear tNorm elvDatNorm angDatNorm rotDatNorm
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
        ax.XLim = [0 100];
        clear ax
        %Place legend
        if aa == 3
            legend(modelLeg,'Location','SouthEast'); legend boxoff
        else
            legend(modelLeg,'Location','NorthWest'); legend boxoff
        end
    end
    clear aa

    %Save figure
    print(['ShoulderAnglesNorm_',(char(taskName)),'_fig.eps'],'-depsc2');        %eps format
    set(gcf, 'PaperPositionMode','auto')
    saveas(gcf,['ShoulderAnglesNorm_',(char(taskName)),'_fig.png']);             %low res png
    saveas(gcf,['ShoulderAnglesNorm_',(char(taskName)),'_fig.fig']);             %matlab figure
    print(gcf,['ShoulderAnglesNorm_',(char(taskName)),'_fig'],'-dtiff','-r600'); %600 dpi tif
    %Close
    close all

    %% Shoulder joint speeds
    
    %Non time normalised
    
    figure; hold on
    %Get current figure position and triple width for the subplot
    f = gcf; figPos = f.Position;
    f.Position = [figPos(1)-figPos(4)*1.25 figPos(2) figPos(3)*3 figPos(4)];
    clear f figPos
    
    %Loop through models and plot
    for mm = 1:length(modelName)        
        %Shoulder elevation
        subplot(1,3,1); hold on
        plot(simResults.(char(taskName)).(modelName{mm}).coordinateSpeed.time,...
            rad2deg(simResults.(char(taskName)).(modelName{mm}).coordinateSpeed.shoulder_elv),...
            'Color',lineColour{mm},'LineWidth',1.5)        
        %Elevation plane
        subplot(1,3,2); hold on
        plot(simResults.(char(taskName)).(modelName{mm}).coordinateSpeed.time,...
            rad2deg(simResults.(char(taskName)).(modelName{mm}).coordinateSpeed.elv_angle),...
            'Color',lineColour{mm},'LineWidth',1.5)        
        %Shoulder rotation
        subplot(1,3,3); hold on
        plot(simResults.(char(taskName)).(modelName{mm}).coordinateSpeed.time,...
            rad2deg(simResults.(char(taskName)).(modelName{mm}).coordinateSpeed.shoulder_rot),...
            'Color',lineColour{mm},'LineWidth',1.5)        
    end
    clear mm
    
    %Find max time. Start with first model
    maxTime = simResults.(char(taskName)).(modelName{1}).coordinateSpeed.time(end);
    for mm = 2:length(modelName)
        if simResults.(char(taskName)).(modelName{mm}).coordinateSpeed.time(end) > maxTime
            maxTime = simResults.(char(taskName)).(modelName{mm}).coordinateSpeed.time(end);
        end            
    end
    
    %Set axes parameters
    for aa = 1:3
        subplot(1,3,aa);
        %Set font characteristics
        set(gca,'FontSize',10,'FontWeight','bold','FontName','Helvetica');
        %Set x axis label
        xlabel('Time (s)',...
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
        ax.XLim = [0 maxTime];
        clear ax
        %Place legend
        if aa == 3
            legend(modelLeg,'Location','NorthEast','NumColumns',2); legend boxoff
        elseif aa == 1
            lgd = legend(modelLeg,'Location','SouthEast'); legend boxoff
            %Shift slightly left
            lgd.Position(1) = lgd.Position(1) - 0.02;
            clear lgd
        else
            legend(modelLeg,'Location','NorthWest'); legend boxoff
        end
    end
    clear aa

    %Save figure
    print(['ShoulderVelocities_',(char(taskName)),'_fig.eps'],'-depsc2');        %eps format
    set(gcf, 'PaperPositionMode','auto')
    saveas(gcf,['ShoulderVelocities_',(char(taskName)),'_fig.png']);             %low res png
    saveas(gcf,['ShoulderVelocities_',(char(taskName)),'_fig.fig']);             %matlab figure
    print(gcf,['ShoulderVelocities_',(char(taskName)),'_fig'],'-dtiff','-r600'); %600 dpi tif
    %Close
    close all
    %Cleanup
    clear maxTime
    
    %Time normalised
    
    figure; hold on
    %Get current figure position and triple width for the subplot
    f = gcf; figPos = f.Position;
    f.Position = [figPos(1)-figPos(4)*1.25 figPos(2) figPos(3)*3 figPos(4)];
    clear f figPos
    
    %Loop through models and plot
    for mm = 1:length(modelName)
        %Create a 101 length version of the time samples
        tNorm = linspace(simResults.(char(taskName)).(modelName{mm}).coordinateSpeed.time(1),...
                simResults.(char(taskName)).(modelName{mm}).coordinateSpeed.time(end),101)';
        %Create normalised versions of shoulder angle data
        elvDatNorm = interp1(simResults.(char(taskName)).(modelName{mm}).coordinateSpeed.time,...
            simResults.(char(taskName)).(modelName{mm}).coordinateSpeed.shoulder_elv,tNorm);
        angDatNorm = interp1(simResults.(char(taskName)).(modelName{mm}).coordinateSpeed.time,...
            simResults.(char(taskName)).(modelName{mm}).coordinateSpeed.elv_angle,tNorm);
        rotDatNorm = interp1(simResults.(char(taskName)).(modelName{mm}).coordinateSpeed.time,...
            simResults.(char(taskName)).(modelName{mm}).coordinateSpeed.shoulder_rot,tNorm);        
        %Shoulder elevation        
        subplot(1,3,1); hold on
        plot(0:100,rad2deg(elvDatNorm),...
            'Color',lineColour{mm},'LineWidth',1.5)        
        %Elevation plane
        subplot(1,3,2); hold on
        plot(0:100,rad2deg(angDatNorm),...
            'Color',lineColour{mm},'LineWidth',1.5)        
        %Shoulder rotation
        subplot(1,3,3); hold on
        plot(0:100,rad2deg(rotDatNorm),...
            'Color',lineColour{mm},'LineWidth',1.5)
        %Cleanup
        clear tNorm elvDatNorm angDatNorm rotDatNorm
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
        ax.XLim = [0 100];
        clear ax
        %Place legend
        if aa == 3
            legend(modelLeg,'Location','NorthEast','NumColumns',2); legend boxoff
        elseif aa == 1
            lgd = legend(modelLeg,'Location','SouthEast'); legend boxoff
            %Shift slightly left
            lgd.Position(1) = lgd.Position(1) - 0.02;
            clear lgd
        else
            legend(modelLeg,'Location','NorthWest'); legend boxoff
        end
    end
    clear aa

    %Save figure
    print(['ShoulderVelocitiesNorm_',(char(taskName)),'_fig.eps'],'-depsc2');        %eps format
    set(gcf, 'PaperPositionMode','auto')
    saveas(gcf,['ShoulderVelocitiesNorm_',(char(taskName)),'_fig.png']);             %low res png
    saveas(gcf,['ShoulderVelocitiesNorm_',(char(taskName)),'_fig.fig']);             %matlab figure
    print(gcf,['ShoulderVelocitiesNorm_',(char(taskName)),'_fig'],'-dtiff','-r600'); %600 dpi tif
    %Close
    close all
    
    %% Muscle activations
    
    %Non time normalised
    
    %Extract a set of strings that represent the muscle activations
    actNames = fieldnames(simResults.(char(taskName)).(modelName{1}).muscleActivation);
    %Remove time label
    actNames = actNames(logical(~contains(actNames,'time')));
    
    %Set a variable for positioning axes on the subplot
    subplotPos = [1:1:24,27,28];
    
    figure; hold on
    %Get current figure position and set size for the subplot
    f = gcf; figPos = f.Position;
    f.Position = [figPos(1)-figPos(4)*1.25 figPos(2)-figPos(3) figPos(3)*3 figPos(4)*2.5];
    clear f figPos
    
    %Loop through models and plot
    for mm = 1:length(modelName)          
        %Loop through activations while placing on appropriate subplot
        for aa = 1:length(actNames)
            subplot(5,6,subplotPos(aa)); hold on
            plot(simResults.(char(taskName)).(modelName{mm}).muscleActivation.time,...
            simResults.(char(taskName)).(modelName{mm}).muscleActivation.(actNames{aa}),...
            'Color',lineColour{mm},'LineWidth',1.5)
        end
        clear aa            
    end
    clear mm
    
    %Find max time. Start with first model
    maxTime = simResults.(char(taskName)).(modelName{1}).muscleActivation.time(end);
    for mm = 2:length(modelName)
        if simResults.(char(taskName)).(modelName{mm}).muscleActivation.time(end) > maxTime
            maxTime = simResults.(char(taskName)).(modelName{mm}).muscleActivation.time(end);
        end            
    end
    
    %Set axes parameters
    for aa = 1:length(subplotPos)
        subplot(5,6,subplotPos(aa));
        %Set font characteristics
        set(gca,'FontSize',9,'FontWeight','bold','FontName','Helvetica');
        %Set x axis label
        xlabel('Time (s)',...
            'FontWeight','bold','FontName','Helvetica',...
            'FontSize',9);
        %Set y axis label
        label = [actNames{aa},' Activation (0-1)'];
        ylabel(label,...
            'FontWeight','bold','FontName','Helvetica',...
            'FontSize',9);    
        clear label
        %Set axes style
        ax = gca; box on; ax.LineWidth = 1;
        set(gca,'Layer','top');
        %Set axes limits
        ax.XLim = [0 maxTime];
        clear ax
    end
    clear aa
    
    %Add legend using custom legendflex function
    ax = subplot(5,6,subplotPos(end-3));
    legendflex(ax, modelLeg, 'anchor', {'s','n'}, 'buffer', [0 -45], 'nrow', length(modelLeg));
    clear ax
    
    %Save figure
    print(['MuscleActivations_',(char(taskName)),'_fig.eps'],'-depsc2');        %eps format
    set(gcf, 'PaperPositionMode','auto')
    saveas(gcf,['MuscleActivations_',(char(taskName)),'_fig.png']);             %low res png
    saveas(gcf,['MuscleActivations_',(char(taskName)),'_fig.fig']);             %matlab figure
    print(gcf,['MuscleActivations_',(char(taskName)),'_fig'],'-dtiff','-r600'); %600 dpi tif
    %Close
    close all
    %Cleanup
    clear maxTime
    
    %Time normalised

    figure; hold on
    %Get current figure position and set size for the subplot
    f = gcf; figPos = f.Position;
    f.Position = [figPos(1)-figPos(4)*1.25 figPos(2)-figPos(3) figPos(3)*3 figPos(4)*2.5];
    clear f figPos
    
    %Loop through models and plot
    for mm = 1:length(modelName)   
        %Create a 101 length version of the time samples
        tNorm = linspace(simResults.(char(taskName)).(modelName{mm}).muscleActivation.time(1),...
            simResults.(char(taskName)).(modelName{mm}).muscleActivation.time(end),101)';        
        %Loop through activations while placing on appropriate subplot
        for aa = 1:length(actNames)
            %Create normalised version of the data
            muscDatNorm = interp1(simResults.(char(taskName)).(modelName{mm}).muscleActivation.time,...
                simResults.(char(taskName)).(modelName{mm}).muscleActivation.(actNames{aa}),tNorm);
            subplot(5,6,subplotPos(aa)); hold on
            plot(0:100,muscDatNorm,'Color',lineColour{mm},'LineWidth',1.5)
            %Cleanup
            clear muscDatNorm
        end
        clear aa tNorm
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
        label = [actNames{aa},' Activation (0-1)'];
        ylabel(label,...
            'FontWeight','bold','FontName','Helvetica',...
            'FontSize',9);    
        clear label
        %Set axes style
        ax = gca; box on; ax.LineWidth = 1;
        set(gca,'Layer','top');
        %Set axes limits
        ax.XLim = [0 100];
        clear ax
    end
    clear aa
    
    %Add legend using custom legendflex function
    ax = subplot(5,6,subplotPos(end-3));
    legendflex(ax, modelLeg, 'anchor', {'s','n'}, 'buffer', [0 -45], 'nrow', length(modelLeg));
    clear ax
    
    %Save figure
    print(['MuscleActivationsNorm_',(char(taskName)),'_fig.eps'],'-depsc2');        %eps format
    set(gcf, 'PaperPositionMode','auto')
    saveas(gcf,['MuscleActivationsNorm_',(char(taskName)),'_fig.png']);             %low res png
    saveas(gcf,['MuscleActivationsNorm_',(char(taskName)),'_fig.fig']);             %matlab figure
    print(gcf,['MuscleActivationsNorm_',(char(taskName)),'_fig'],'-dtiff','-r600'); %600 dpi tif
    %Close
    close all
    
    %% Muscle cost
    
    %Set a labelling variable for plot
    muscleGroupsLabels = [{'Scapula Protractors'};
        {'External Rotators'};
        {'Scapula Retractors'};
        {'Abductors'};
        {'Internal Rotators'};
        {'Flexors'};
        {'Horizontal Adductors'}];
    
    figure; hold on
    %Get current figure position and double width 
    f = gcf; figPos = f.Position;
    f.Position = [figPos(1)-figPos(4) figPos(2) figPos(3)*2.5 figPos(4)];
    clear f figPos
    
    %Plot normalised muscle cost values for all models (except for None)
    for mm = 2:length(modelName)
        b = bar(mm-1,simResults.(char(taskName)).(modelName{mm}).muscleCostTotalNorm.all);
        b.FaceColor = lineColour{mm}; b.EdgeColor = [0,0,0]; b.BarWidth = 1; b.LineWidth = 1;
    end
    clear mm
    
    %Plot muscle groupings as subsequent datasets
    for gg = 1:length(muscleGroups)
        for mm = 2:length(modelName)
            b = bar(mm+(gg*length(modelName))+(gg-1),simResults.(char(taskName)).(modelName{mm}).muscleCostTotalNorm.(muscleGroups{gg}));
            b.FaceColor = lineColour{mm}; b.EdgeColor = [0,0,0]; b.BarWidth = 1; b.LineWidth = 1;
        end
        clear mm
    end
    clear gg
    
    %Get axes
    ax = gca;
    
    %Set X tick points and labels
    ax.XTick = length(modelName)/2:length(modelName)+1:(length(modelName)+1)*length(muscleGroups)+(length(modelName)/2);
    ax.XTickLabel = [{'All Muscles'};muscleGroupsLabels];
    ax.XTickLabelRotation = 45;
    ax.TickLength = ax.TickLength/2;
    
    %Set axes parameters
    %Set font characteristics
    set(gca,'FontSize',10,'FontWeight','bold','FontName','Helvetica');
    %Set y axis label
    ylabel('% Change in Total Muscle Cost',...
        'FontWeight','bold','FontName','Helvetica',...
        'FontSize',10);    
    %Set axes style
    ax = gca; box on; ax.LineWidth = 1;
    set(gca,'Layer','top');
    %Set legend
    legend(modelLeg{2:end},'Location','SouthEast','NumColumns',1); legend box off

    %Save figure
    print(['MuscleCostChange_',(char(taskName)),'_fig.eps'],'-depsc2');        %eps format
    set(gcf, 'PaperPositionMode','auto')
    saveas(gcf,['MuscleCostChange_',(char(taskName)),'_fig.png']);             %low res png
    saveas(gcf,['MuscleCostChange_',(char(taskName)),'_fig.fig']);             %matlab figure
    print(gcf,['MuscleCostChange_',(char(taskName)),'_fig'],'-dtiff','-r600'); %600 dpi tif
    %Close
    close all  

%%

%%%%% TO DO: also consider figure averaging the impact across tasks for
%%%%% muscle cost (i.e. mean +/- SD) to get overall effect...


end
