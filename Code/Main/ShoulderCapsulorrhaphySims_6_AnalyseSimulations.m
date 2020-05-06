function ShoulderCapsulorrhaphySims_6_AnalyseSimulations

% @author: Aaron Fox
% Centre for Sport Research, Deakin University
% aaron.f@deakin.edu.au
% 
% This code extracts and analyses the results of the simulated tasks

    import org.opensim.modeling.*
    warning off

    %Set main directory
    mainDir = pwd;

    %Add supplementary code folder to path
    addpath(genpath('..\Supplementary'));
    
    %Load the dual expression based force plugin library
    cd('..\..\Plugin_DualEBCF\build\Release');
    opensimCommon.LoadOpenSimLibraryExact([pwd,'\osimDualEBCF.dll']);
    
    %Set a variable for tasks to be analysed
    simTasks = [{'ConcentricUpwardReach105'};
        {'ConcentricForwardReach'};
        {'HairTouch'}];
    
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
    cd('..\..\..\ModelFiles'); modelsDir = [pwd,'\'];
    ModelVisualizer.addDirToGeometrySearchPaths([modelsDir,'Geometry']);
    for mm = 1:length(modelName)
        modelFile{mm,1} = [modelsDir,'FullShoulderModel_',modelName{mm},'.osim'];        
    end
    clear mm
    
    %Set colour scheme variables for the nine conditions (for later plots)
    lineColour = [{'#000000'};
        {'#4885ed'};
        {'#f4c20d'};
        {'#db3236'};
        {'#d56dd7'};
        {'#1e9c31'};
        {'#3be2ff'};
        {'#9c661c'};
        {'#8c1b7f'}];

    %Create legend for model names (for later plots)
    modelLeg = [{'None'};
        {'Anteroinferior'};
        {'Anterosuperior'};
        {'Posteroinferior'};
        {'Posterosuperior'};
        {'Total Anterior'};
        {'Total Inferior'};
        {'Total Posterior'};
        {'Total Superior'}];
    
    %Navigate to results directory
    cd('..\SimulationResults');
    resultsDir = [pwd,'\'];

    %% Extract simulation results
    
    for tt = 1:length(simTasks)
        
        %Set current task
        taskName = simTasks{tt};
    
        %Navigate to results path
        cd(taskName);
        
        %Identify the files that contain the current task name
        f = dir(['*',taskName,'*']);
        for ff = 1:length(f)
            taskFiles{ff,1} = f(ff).name;
        end
        clear f ff

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
            
            %%%%% Below is commented out to avoid running analyses
            %%%%% simulations each time script is tested...

% % %             %Note that the below analysis pipeline is a little convoluted, as
% % %             %the only way to seemingly not have Matlab crash is to generate the
% % %             %tool, print it out, load it back in and then run it - weird, but
% % %             %this process works...
% % % 
% % %             %File paths are getting too long and hence the task name can't be
% % %             %included in the analysis files, otherwise the longer ones don't
% % %             %print out
% % % 
% % %             %Run a muscle analysis on Moco output
% % % 
% % %             %Initialise a blank analysis tool
% % %             AnTool = AnalyzeTool();
% % % 
% % %             %Provide inputs to analysis tool
% % %             %Model
% % %             AnTool.setModelFilename(modelFile{mm});
% % %             %States
% % %             AnTool.setStatesFileName(fileName{1,1});
% % %             %Time range
% % %             AnTool.setStartTime(currSolution.getInitialTime());
% % %             AnTool.setFinalTime(currSolution.getFinalTime());
% % %             %Solve for equilibrium
% % %             AnTool.setSolveForEquilibrium(true)
% % %             %Results directory
% % %             AnTool.setResultsDir(pwd);
% % %             %Tool name
% % %             AnTool.setName(modelName{mm});
% % % 
% % %             %Intialise force reporter analysis set
% % %             FrAnalysis = ForceReporter();
% % % 
% % %             %Provide inputs to force reporter analysis
% % %             %Set name
% % %             FrAnalysis.setName('ForceReporter');
% % %             %Set start and end time
% % %             FrAnalysis.setStartTime(currSolution.getInitialTime());
% % %             FrAnalysis.setEndTime(currSolution.getFinalTime());
% % %             %Options
% % %             FrAnalysis.setStepInterval(1);
% % %             FrAnalysis.setInDegrees(true);
% % % 
% % %             %Add the force reporter analysis to the analyse tool
% % %             AnTool.getAnalysisSet().cloneAndAppend(FrAnalysis);
% % % 
% % %             %Initialise muscle analysis tool
% % %             MaAnalysis = MuscleAnalysis();
% % % 
% % %             %Provide inputs to muscle analysis
% % %             %Set name
% % %             MaAnalysis.setName('MuscleAnalysis');
% % %             %Set start and end time
% % %             MaAnalysis.setStartTime(currSolution.getInitialTime());
% % %             MaAnalysis.setEndTime(currSolution.getFinalTime());
% % %             %Options
% % %             MaAnalysis.setStepInterval(1);
% % %             MaAnalysis.setInDegrees(true);
% % % 
% % %             %Add the muscle analysis to the analyse tool
% % %             AnTool.getAnalysisSet().cloneAndAppend(MaAnalysis);
% % % 
% % %             %Print tool to file
% % %             AnTool.print([modelName{mm},'_analysis.xml']);
% % % 
% % %             %Re-import tool back in
% % %             runTool = AnalyzeTool([modelName{mm},'_analysis.xml']);
% % % 
% % %             %Run analysis tool
% % %             clc; runTool.run();
% % % 
% % %             %Cleanup existing files
% % %             delete([modelName{mm},'_analysis.xml']);

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

        %% Performance time

        %%%% TO DO: could also time normalise performance to the None model...
        
        %%%%% Plotting of results commented out below to avoid generating
        %%%%% figures on each test iteration of the script...

% % %         figure; hold on
% % % 
% % %         %Extract performance time into relevant structure
% % %         for mm = 1:length(modelName)
% % %             simResults.(char(taskName)).(modelName{mm}).perfTime = ...
% % %                 simResults.(char(taskName)).(modelName{mm}).coordinateValue.time(end);
% % %             %Store variable for plotting
% % %             perfTime(mm,1) = simResults.(char(taskName)).(modelName{mm}).coordinateValue.time(end);
% % %         end
% % %         clear mm
% % % 
% % %         %Create bar chart for performance time
% % %         for pp = 1:length(perfTime)
% % %             b = bar(pp,perfTime(pp));
% % %             b.FaceColor = lineColour{pp}; b.EdgeColor = [0,0,0]; b.BarWidth = 1; b.LineWidth = 1;
% % %         end
% % %         clear pp
% % %        
% % %         %Set formatting on axes numbers
% % %         set(gca,'FontSize',10,'FontWeight','bold','FontName','Helvetica');
% % %         ax = gca; box on; ax.LineWidth = 1;
% % %         set(gca,'Layer','top');
% % %         %Set axes limits and labels
% % %         ax.XLim = [0.3 9.7]; ax.YLim(1) = 0;
% % %         for mm = 1:length(modelName)
% % %             ax.XTickLabel{mm} = modelLeg{mm};
% % %         end
% % %         ax.XTickLabelRotation = 45;
% % %         ylabel('Task Performance Time (s)',...
% % %             'FontWeight','bold','FontName','Helvetica',...
% % %             'FontSize',12);
% % %         
% % %         %Save figure
% % %         print(['PerformanceTime_',(char(taskName)),'_fig.eps'],'-depsc2');        %eps format
% % %         set(gcf, 'PaperPositionMode','auto')
% % %         saveas(gcf,['PerformanceTime_',(char(taskName)),'_fig.png']);             %low res png
% % %         saveas(gcf,['PerformanceTime_',(char(taskName)),'_fig.fig']);             %matlab figure
% % %         print(gcf,['PerformanceTime_',(char(taskName)),'_fig'],'-dtiff','-r600'); %600 dpi tif
% % %         %Close
% % %         close all
% % %         
% % %         %% Shoulder joint angles
% % % 
% % %         %Non-time normalised
% % % 
% % %         figure; hold on
% % %         %Get current figure position and triple width for the subplot
% % %         f = gcf; figPos = f.Position;
% % %         f.Position = [figPos(1)-figPos(4)*1.25 figPos(2) figPos(3)*3 figPos(4)];
% % %         clear f figPos
% % % 
% % %         %Loop through models and plot
% % %         for mm = 1:length(modelName)        
% % %             %Shoulder elevation
% % %             subplot(1,3,1); hold on
% % %             plot(simResults.(char(taskName)).(modelName{mm}).coordinateValue.time,...
% % %                 rad2deg(simResults.(char(taskName)).(modelName{mm}).coordinateValue.shoulder_elv),...
% % %                 'Color',lineColour{mm},'LineWidth',1.5)        
% % %             %Elevation plane
% % %             subplot(1,3,2); hold on
% % %             plot(simResults.(char(taskName)).(modelName{mm}).coordinateValue.time,...
% % %                 rad2deg(simResults.(char(taskName)).(modelName{mm}).coordinateValue.elv_angle),...
% % %                 'Color',lineColour{mm},'LineWidth',1.5)        
% % %             %Shoulder rotation
% % %             subplot(1,3,3); hold on
% % %             plot(simResults.(char(taskName)).(modelName{mm}).coordinateValue.time,...
% % %                 rad2deg(simResults.(char(taskName)).(modelName{mm}).coordinateValue.shoulder_rot),...
% % %                 'Color',lineColour{mm},'LineWidth',1.5)        
% % %         end
% % %         clear mm
% % % 
% % %         %Find max time. Start with first model
% % %         maxTime = simResults.(char(taskName)).(modelName{1}).coordinateValue.time(end);
% % %         for mm = 2:length(modelName)
% % %             if simResults.(char(taskName)).(modelName{mm}).coordinateValue.time(end) > maxTime
% % %                 maxTime = simResults.(char(taskName)).(modelName{mm}).coordinateValue.time(end);
% % %             end            
% % %         end
% % % 
% % %         %Set axes parameters
% % %         for aa = 1:3
% % %             subplot(1,3,aa);
% % %             %Set font characteristics
% % %             set(gca,'FontSize',10,'FontWeight','bold','FontName','Helvetica');
% % %             %Set x axis label
% % %             xlabel('Time (s)',...
% % %                 'FontWeight','bold','FontName','Helvetica',...
% % %                 'FontSize',12);
% % %             %Set y axis label
% % %             if aa == 1
% % %                 label = ['Shoulder Elevation (',char(176),')'];
% % %             elseif aa == 2
% % %                 label = ['Elevation Plane (',char(176),')'];
% % %             elseif aa == 3
% % %                 label = ['Shoulder Rotation (',char(176),')'];
% % %             end
% % %             ylabel(label,...
% % %                 'FontWeight','bold','FontName','Helvetica',...
% % %                 'FontSize',12);    
% % %             clear label
% % %             %Set axes style
% % %             ax = gca; box on; ax.LineWidth = 1;
% % %             set(gca,'Layer','top');
% % %             %Set axes limits        
% % %             ax.XLim = [0 maxTime];
% % %             clear ax
% % %             %Place legend
% % %             if tt == 3 && aa == 2
% % %                 %First adjust axis to fit legend
% % %                 ax = gca; ax.YLim(2) = 26.8;
% % %                 legend(modelLeg,'Location','NorthEast'); legend boxoff            
% % %             elseif aa == 3
% % %                 if tt == 3
% % %                     legend(modelLeg,'Location','SouthWest'); legend boxoff
% % %                 else
% % %                     legend(modelLeg,'Location','SouthEast'); legend boxoff
% % %                 end
% % %             else
% % %                 legend(modelLeg,'Location','NorthWest'); legend boxoff
% % %             end
% % %         end
% % %         clear aa
% % % 
% % %         %Save figure
% % %         print(['ShoulderAngles_',(char(taskName)),'_fig.eps'],'-depsc2');        %eps format
% % %         set(gcf, 'PaperPositionMode','auto')
% % %         saveas(gcf,['ShoulderAngles_',(char(taskName)),'_fig.png']);             %low res png
% % %         saveas(gcf,['ShoulderAngles_',(char(taskName)),'_fig.fig']);             %matlab figure
% % %         print(gcf,['ShoulderAngles_',(char(taskName)),'_fig'],'-dtiff','-r600'); %600 dpi tif
% % %         %Close
% % %         close all
% % %         %Cleanup
% % %         clear maxTime
% % % 
% % %         %Time normalised
% % % 
% % %         figure; hold on
% % %         %Get current figure position and triple width for the subplot
% % %         f = gcf; figPos = f.Position;
% % %         f.Position = [figPos(1)-figPos(4)*1.25 figPos(2) figPos(3)*3 figPos(4)];
% % %         clear f figPos
% % % 
% % %         %Loop through models and plot
% % %         for mm = 1:length(modelName)  
% % %             %Create a 101 length version of the time samples
% % %             tNorm = linspace(simResults.(char(taskName)).(modelName{mm}).coordinateValue.time(1),...
% % %                     simResults.(char(taskName)).(modelName{mm}).coordinateValue.time(end),101)';
% % %             %Create normalised versions of shoulder angle data
% % %             elvDatNorm = interp1(simResults.(char(taskName)).(modelName{mm}).coordinateValue.time,...
% % %                 simResults.(char(taskName)).(modelName{mm}).coordinateValue.shoulder_elv,tNorm);
% % %             angDatNorm = interp1(simResults.(char(taskName)).(modelName{mm}).coordinateValue.time,...
% % %                 simResults.(char(taskName)).(modelName{mm}).coordinateValue.elv_angle,tNorm);
% % %             rotDatNorm = interp1(simResults.(char(taskName)).(modelName{mm}).coordinateValue.time,...
% % %                 simResults.(char(taskName)).(modelName{mm}).coordinateValue.shoulder_rot,tNorm);
% % %             %Shoulder elevation        
% % %             subplot(1,3,1); hold on
% % %             plot(0:100,rad2deg(elvDatNorm),...
% % %                 'Color',lineColour{mm},'LineWidth',1.5)        
% % %             %Elevation plane
% % %             subplot(1,3,2); hold on
% % %             plot(0:100,rad2deg(angDatNorm),...
% % %                 'Color',lineColour{mm},'LineWidth',1.5)        
% % %             %Shoulder rotation
% % %             subplot(1,3,3); hold on
% % %             plot(0:100,rad2deg(rotDatNorm),...
% % %                 'Color',lineColour{mm},'LineWidth',1.5)
% % %             %Cleanup
% % %             clear tNorm elvDatNorm angDatNorm rotDatNorm
% % %         end
% % %         clear mm
% % % 
% % %         %Set axes parameters
% % %         for aa = 1:3
% % %             subplot(1,3,aa);
% % %             %Set font characteristics
% % %             set(gca,'FontSize',10,'FontWeight','bold','FontName','Helvetica');
% % %             %Set x axis label
% % %             xlabel('0-100% Task Completion',...
% % %                 'FontWeight','bold','FontName','Helvetica',...
% % %                 'FontSize',12);
% % %             %Set y axis label
% % %             if aa == 1
% % %                 label = ['Shoulder Elevation (',char(176),')'];
% % %             elseif aa == 2
% % %                 label = ['Elevation Plane (',char(176),')'];
% % %             elseif aa == 3
% % %                 label = ['Shoulder Rotation (',char(176),')'];
% % %             end
% % %             ylabel(label,...
% % %                 'FontWeight','bold','FontName','Helvetica',...
% % %                 'FontSize',12);    
% % %             clear label
% % %             %Set axes style
% % %             ax = gca; box on; ax.LineWidth = 1;
% % %             set(gca,'Layer','top');
% % %             %Set axes limits        
% % %             ax.XLim = [0 100];
% % %             clear ax
% % %             %Place legend
% % %             if tt == 3 && aa == 2
% % %                 %First adjust axis to fit legend
% % %                 ax = gca; ax.YLim(2) = 26.8;
% % %                 legend(modelLeg,'Location','NorthEast'); legend boxoff            
% % %             elseif aa == 3
% % %                 if tt == 3
% % %                     legend(modelLeg,'Location','SouthWest'); legend boxoff
% % %                 else
% % %                     legend(modelLeg,'Location','SouthEast'); legend boxoff
% % %                 end
% % %             else
% % %                 legend(modelLeg,'Location','NorthWest'); legend boxoff
% % %             end
% % %         end
% % %         clear aa
% % % 
% % %         %Save figure
% % %         print(['ShoulderAnglesNorm_',(char(taskName)),'_fig.eps'],'-depsc2');        %eps format
% % %         set(gcf, 'PaperPositionMode','auto')
% % %         saveas(gcf,['ShoulderAnglesNorm_',(char(taskName)),'_fig.png']);             %low res png
% % %         saveas(gcf,['ShoulderAnglesNorm_',(char(taskName)),'_fig.fig']);             %matlab figure
% % %         print(gcf,['ShoulderAnglesNorm_',(char(taskName)),'_fig'],'-dtiff','-r600'); %600 dpi tif
% % %         %Close
% % %         close all
% % % 
% % %         %% Shoulder joint speeds
% % % 
% % %         %Non time normalised
% % % 
% % %         figure; hold on
% % %         %Get current figure position and triple width for the subplot
% % %         f = gcf; figPos = f.Position;
% % %         f.Position = [figPos(1)-figPos(4)*1.25 figPos(2) figPos(3)*3 figPos(4)];
% % %         clear f figPos
% % % 
% % %         %Loop through models and plot
% % %         for mm = 1:length(modelName)        
% % %             %Shoulder elevation
% % %             subplot(1,3,1); hold on
% % %             plot(simResults.(char(taskName)).(modelName{mm}).coordinateSpeed.time,...
% % %                 rad2deg(simResults.(char(taskName)).(modelName{mm}).coordinateSpeed.shoulder_elv),...
% % %                 'Color',lineColour{mm},'LineWidth',1.5)        
% % %             %Elevation plane
% % %             subplot(1,3,2); hold on
% % %             plot(simResults.(char(taskName)).(modelName{mm}).coordinateSpeed.time,...
% % %                 rad2deg(simResults.(char(taskName)).(modelName{mm}).coordinateSpeed.elv_angle),...
% % %                 'Color',lineColour{mm},'LineWidth',1.5)        
% % %             %Shoulder rotation
% % %             subplot(1,3,3); hold on
% % %             plot(simResults.(char(taskName)).(modelName{mm}).coordinateSpeed.time,...
% % %                 rad2deg(simResults.(char(taskName)).(modelName{mm}).coordinateSpeed.shoulder_rot),...
% % %                 'Color',lineColour{mm},'LineWidth',1.5)        
% % %         end
% % %         clear mm
% % % 
% % %         %Find max time. Start with first model
% % %         maxTime = simResults.(char(taskName)).(modelName{1}).coordinateSpeed.time(end);
% % %         for mm = 2:length(modelName)
% % %             if simResults.(char(taskName)).(modelName{mm}).coordinateSpeed.time(end) > maxTime
% % %                 maxTime = simResults.(char(taskName)).(modelName{mm}).coordinateSpeed.time(end);
% % %             end            
% % %         end
% % % 
% % %         %Set axes parameters
% % %         for aa = 1:3
% % %             subplot(1,3,aa);
% % %             %Set font characteristics
% % %             set(gca,'FontSize',10,'FontWeight','bold','FontName','Helvetica');
% % %             %Set x axis label
% % %             xlabel('Time (s)',...
% % %                 'FontWeight','bold','FontName','Helvetica',...
% % %                 'FontSize',12);
% % %             %Set y axis label
% % %             if aa == 1
% % %                 label = ['Shoulder Elevation Velocity (',char(176),'^{.}s^{-1})'];
% % %             elseif aa == 2
% % %                 label = ['Elevation Plane Velocity (',char(176),'^{.}s^{-1})'];
% % %             elseif aa == 3
% % %                 label = ['Shoulder Rotation Velocity (',char(176),'^{.}s^{-1})'];
% % %             end
% % %             ylabel(label,...
% % %                 'FontWeight','bold','FontName','Helvetica',...
% % %                 'FontSize',12);    
% % %             clear label
% % %             %Set axes style
% % %             ax = gca; box on; ax.LineWidth = 1;
% % %             set(gca,'Layer','top');
% % %             %Set axes limits        
% % %             ax.XLim = [0 maxTime];
% % %             clear ax
% % %             %Place legend
% % %             if tt == 3
% % %                 if aa == 1
% % %                     lgd = legend(modelLeg,'Location','SouthEast'); legend boxoff
% % %                     %Shift slightly left
% % %                     lgd.Position(1) = lgd.Position(1) - 0.025;
% % %                     clear lgd
% % %                 elseif aa == 2
% % %                     legend(modelLeg,'Location','NorthEast'); legend boxoff
% % %                 else
% % %                     legend(modelLeg,'Location','NorthEast','NumColumns',1); legend boxoff
% % %                 end
% % %             elseif aa == 3
% % %                 legend(modelLeg,'Location','NorthEast','NumColumns',2); legend boxoff
% % %             elseif aa == 1
% % %                 lgd = legend(modelLeg,'Location','SouthEast'); legend boxoff
% % %                 %Shift slightly left
% % %                 lgd.Position(1) = lgd.Position(1) - 0.02;
% % %                 clear lgd
% % %             else
% % %                 legend(modelLeg,'Location','NorthWest'); legend boxoff
% % %             end
% % %         end
% % %         clear aa
% % % 
% % %         %Save figure
% % %         print(['ShoulderVelocities_',(char(taskName)),'_fig.eps'],'-depsc2');        %eps format
% % %         set(gcf, 'PaperPositionMode','auto')
% % %         saveas(gcf,['ShoulderVelocities_',(char(taskName)),'_fig.png']);             %low res png
% % %         saveas(gcf,['ShoulderVelocities_',(char(taskName)),'_fig.fig']);             %matlab figure
% % %         print(gcf,['ShoulderVelocities_',(char(taskName)),'_fig'],'-dtiff','-r600'); %600 dpi tif
% % %         %Close
% % %         close all
% % %         %Cleanup
% % %         clear maxTime
% % % 
% % %         %Time normalised
% % % 
% % %         figure; hold on
% % %         %Get current figure position and triple width for the subplot
% % %         f = gcf; figPos = f.Position;
% % %         f.Position = [figPos(1)-figPos(4)*1.25 figPos(2) figPos(3)*3 figPos(4)];
% % %         clear f figPos
% % % 
% % %         %Loop through models and plot
% % %         for mm = 1:length(modelName)
% % %             %Create a 101 length version of the time samples
% % %             tNorm = linspace(simResults.(char(taskName)).(modelName{mm}).coordinateSpeed.time(1),...
% % %                     simResults.(char(taskName)).(modelName{mm}).coordinateSpeed.time(end),101)';
% % %             %Create normalised versions of shoulder angle data
% % %             elvDatNorm = interp1(simResults.(char(taskName)).(modelName{mm}).coordinateSpeed.time,...
% % %                 simResults.(char(taskName)).(modelName{mm}).coordinateSpeed.shoulder_elv,tNorm);
% % %             angDatNorm = interp1(simResults.(char(taskName)).(modelName{mm}).coordinateSpeed.time,...
% % %                 simResults.(char(taskName)).(modelName{mm}).coordinateSpeed.elv_angle,tNorm);
% % %             rotDatNorm = interp1(simResults.(char(taskName)).(modelName{mm}).coordinateSpeed.time,...
% % %                 simResults.(char(taskName)).(modelName{mm}).coordinateSpeed.shoulder_rot,tNorm);        
% % %             %Shoulder elevation        
% % %             subplot(1,3,1); hold on
% % %             plot(0:100,rad2deg(elvDatNorm),...
% % %                 'Color',lineColour{mm},'LineWidth',1.5)        
% % %             %Elevation plane
% % %             subplot(1,3,2); hold on
% % %             plot(0:100,rad2deg(angDatNorm),...
% % %                 'Color',lineColour{mm},'LineWidth',1.5)        
% % %             %Shoulder rotation
% % %             subplot(1,3,3); hold on
% % %             plot(0:100,rad2deg(rotDatNorm),...
% % %                 'Color',lineColour{mm},'LineWidth',1.5)
% % %             %Cleanup
% % %             clear tNorm elvDatNorm angDatNorm rotDatNorm
% % %         end
% % %         clear mm
% % % 
% % %         %Set axes parameters
% % %         for aa = 1:3
% % %             subplot(1,3,aa);
% % %             %Set font characteristics
% % %             set(gca,'FontSize',10,'FontWeight','bold','FontName','Helvetica');
% % %             %Set x axis label
% % %             xlabel('0-100% Task Completion',...
% % %                 'FontWeight','bold','FontName','Helvetica',...
% % %                 'FontSize',12);
% % %             %Set y axis label
% % %             if aa == 1
% % %                 label = ['Shoulder Elevation Velocity (',char(176),'^{.}s^{-1})'];
% % %             elseif aa == 2
% % %                 label = ['Elevation Plane Velocity (',char(176),'^{.}s^{-1})'];
% % %             elseif aa == 3
% % %                 label = ['Shoulder Rotation Velocity (',char(176),'^{.}s^{-1})'];
% % %             end
% % %             ylabel(label,...
% % %                 'FontWeight','bold','FontName','Helvetica',...
% % %                 'FontSize',12);    
% % %             clear label
% % %             %Set axes style
% % %             ax = gca; box on; ax.LineWidth = 1;
% % %             set(gca,'Layer','top');
% % %             %Set axes limits        
% % %             ax.XLim = [0 100];
% % %             clear ax
% % %             %Place legend
% % %             if tt == 3
% % %                 if aa == 1
% % %                     lgd = legend(modelLeg,'Location','SouthEast'); legend boxoff
% % %                     %Shift slightly left
% % %                     lgd.Position(1) = lgd.Position(1) - 0.025;
% % %                     clear lgd
% % %                 elseif aa == 2
% % %                     legend(modelLeg,'Location','NorthEast'); legend boxoff
% % %                 else
% % %                     legend(modelLeg,'Location','NorthEast','NumColumns',1); legend boxoff
% % %                 end
% % %             elseif aa == 3
% % %                 legend(modelLeg,'Location','NorthEast','NumColumns',2); legend boxoff
% % %             elseif aa == 1
% % %                 lgd = legend(modelLeg,'Location','SouthEast'); legend boxoff
% % %                 %Shift slightly left
% % %                 lgd.Position(1) = lgd.Position(1) - 0.02;
% % %                 clear lgd
% % %             else
% % %                 legend(modelLeg,'Location','NorthWest'); legend boxoff
% % %             end
% % %         end
% % %         clear aa
% % % 
% % %         %Save figure
% % %         print(['ShoulderVelocitiesNorm_',(char(taskName)),'_fig.eps'],'-depsc2');        %eps format
% % %         set(gcf, 'PaperPositionMode','auto')
% % %         saveas(gcf,['ShoulderVelocitiesNorm_',(char(taskName)),'_fig.png']);             %low res png
% % %         saveas(gcf,['ShoulderVelocitiesNorm_',(char(taskName)),'_fig.fig']);             %matlab figure
% % %         print(gcf,['ShoulderVelocitiesNorm_',(char(taskName)),'_fig'],'-dtiff','-r600'); %600 dpi tif
% % %         %Close
% % %         close all
% % % 
% % %         %% Muscle activations
% % % 
% % %         %Non time normalised
% % % 
% % %         %Extract a set of strings that represent the muscle activations
% % %         actNames = fieldnames(simResults.(char(taskName)).(modelName{1}).muscleActivation);
% % %         %Remove time label
% % %         actNames = actNames(logical(~contains(actNames,'time')));
% % % 
% % %         %Set a variable for positioning axes on the subplot
% % %         subplotPos = [1:1:24,27,28];
% % % 
% % %         figure; hold on
% % %         %Get current figure position and set size for the subplot
% % %         f = gcf; figPos = f.Position;
% % %         f.Position = [figPos(1)-figPos(4)*1.25 figPos(2)-figPos(3) figPos(3)*3 figPos(4)*2.5];
% % %         clear f figPos
% % % 
% % %         %Loop through models and plot
% % %         for mm = 1:length(modelName)          
% % %             %Loop through activations while placing on appropriate subplot
% % %             for aa = 1:length(actNames)
% % %                 subplot(5,6,subplotPos(aa)); hold on
% % %                 plot(simResults.(char(taskName)).(modelName{mm}).muscleActivation.time,...
% % %                 simResults.(char(taskName)).(modelName{mm}).muscleActivation.(actNames{aa}),...
% % %                 'Color',lineColour{mm},'LineWidth',1.5)
% % %             end
% % %             clear aa            
% % %         end
% % %         clear mm
% % % 
% % %         %Find max time. Start with first model
% % %         maxTime = simResults.(char(taskName)).(modelName{1}).muscleActivation.time(end);
% % %         for mm = 2:length(modelName)
% % %             if simResults.(char(taskName)).(modelName{mm}).muscleActivation.time(end) > maxTime
% % %                 maxTime = simResults.(char(taskName)).(modelName{mm}).muscleActivation.time(end);
% % %             end            
% % %         end
% % % 
% % %         %Set axes parameters
% % %         for aa = 1:length(subplotPos)
% % %             subplot(5,6,subplotPos(aa));
% % %             %Set font characteristics
% % %             set(gca,'FontSize',9,'FontWeight','bold','FontName','Helvetica');
% % %             %Set x axis label
% % %             xlabel('Time (s)',...
% % %                 'FontWeight','bold','FontName','Helvetica',...
% % %                 'FontSize',9);
% % %             %Set y axis label
% % %             label = [actNames{aa},' Activation (0-1)'];
% % %             ylabel(label,...
% % %                 'FontWeight','bold','FontName','Helvetica',...
% % %                 'FontSize',9);    
% % %             clear label
% % %             %Set axes style
% % %             ax = gca; box on; ax.LineWidth = 1;
% % %             set(gca,'Layer','top');
% % %             %Set axes limits
% % %             ax.XLim = [0 maxTime];
% % %             clear ax
% % %         end
% % %         clear aa
% % % 
% % %         %Add legend using custom legendflex function
% % %         ax = subplot(5,6,subplotPos(end-3));
% % %         legendflex(ax, modelLeg, 'anchor', {'s','n'}, 'buffer', [0 -45], 'nrow', length(modelLeg));
% % %         clear ax
% % % 
% % %         %Save figure
% % %         print(['MuscleActivations_',(char(taskName)),'_fig.eps'],'-depsc2');        %eps format
% % %         set(gcf, 'PaperPositionMode','auto')
% % %         saveas(gcf,['MuscleActivations_',(char(taskName)),'_fig.png']);             %low res png
% % %         saveas(gcf,['MuscleActivations_',(char(taskName)),'_fig.fig']);             %matlab figure
% % %         print(gcf,['MuscleActivations_',(char(taskName)),'_fig'],'-dtiff','-r600'); %600 dpi tif
% % %         %Close
% % %         close all
% % %         %Cleanup
% % %         clear maxTime
% % % 
% % %         %Time normalised
% % % 
% % %         figure; hold on
% % %         %Get current figure position and set size for the subplot
% % %         f = gcf; figPos = f.Position;
% % %         f.Position = [figPos(1)-figPos(4)*1.25 figPos(2)-figPos(3) figPos(3)*3 figPos(4)*2.5];
% % %         clear f figPos
% % % 
% % %         %Loop through models and plot
% % %         for mm = 1:length(modelName)   
% % %             %Create a 101 length version of the time samples
% % %             tNorm = linspace(simResults.(char(taskName)).(modelName{mm}).muscleActivation.time(1),...
% % %                 simResults.(char(taskName)).(modelName{mm}).muscleActivation.time(end),101)';        
% % %             %Loop through activations while placing on appropriate subplot
% % %             for aa = 1:length(actNames)
% % %                 %Create normalised version of the data
% % %                 muscDatNorm = interp1(simResults.(char(taskName)).(modelName{mm}).muscleActivation.time,...
% % %                     simResults.(char(taskName)).(modelName{mm}).muscleActivation.(actNames{aa}),tNorm);
% % %                 subplot(5,6,subplotPos(aa)); hold on
% % %                 plot(0:100,muscDatNorm,'Color',lineColour{mm},'LineWidth',1.5)
% % %                 %Cleanup
% % %                 clear muscDatNorm
% % %             end
% % %             clear aa tNorm
% % %         end
% % %         clear mm
% % % 
% % %         %Set axes parameters
% % %         for aa = 1:length(subplotPos)
% % %             subplot(5,6,subplotPos(aa));
% % %             %Set font characteristics
% % %             set(gca,'FontSize',9,'FontWeight','bold','FontName','Helvetica');
% % %             %Set x axis label
% % %             xlabel('0-100% Task Completion',...
% % %                 'FontWeight','bold','FontName','Helvetica',...
% % %                 'FontSize',9);
% % %             %Set y axis label
% % %             label = [actNames{aa},' Activation (0-1)'];
% % %             ylabel(label,...
% % %                 'FontWeight','bold','FontName','Helvetica',...
% % %                 'FontSize',9);    
% % %             clear label
% % %             %Set axes style
% % %             ax = gca; box on; ax.LineWidth = 1;
% % %             set(gca,'Layer','top');
% % %             %Set axes limits
% % %             ax.XLim = [0 100];
% % %             clear ax
% % %         end
% % %         clear aa
% % % 
% % %         %Add legend using custom legendflex function
% % %         ax = subplot(5,6,subplotPos(end-3));
% % %         legendflex(ax, modelLeg, 'anchor', {'s','n'}, 'buffer', [0 -45], 'nrow', length(modelLeg));
% % %         clear ax
% % % 
% % %         %Save figure
% % %         print(['MuscleActivationsNorm_',(char(taskName)),'_fig.eps'],'-depsc2');        %eps format
% % %         set(gcf, 'PaperPositionMode','auto')
% % %         saveas(gcf,['MuscleActivationsNorm_',(char(taskName)),'_fig.png']);             %low res png
% % %         saveas(gcf,['MuscleActivationsNorm_',(char(taskName)),'_fig.fig']);             %matlab figure
% % %         print(gcf,['MuscleActivationsNorm_',(char(taskName)),'_fig'],'-dtiff','-r600'); %600 dpi tif
% % %         %Close
% % %         close all
% % %         
% % %         %Time and scale normalised
% % % 
% % %         figure; hold on
% % %         %Get current figure position and set size for the subplot
% % %         f = gcf; figPos = f.Position;
% % %         f.Position = [figPos(1)-figPos(4)*1.25 figPos(2)-figPos(3) figPos(3)*3 figPos(4)*2.5];
% % %         clear f figPos
% % % 
% % %         %Loop through models and plot
% % %         for mm = 1:length(modelName)   
% % %             %Create a 101 length version of the time samples
% % %             tNorm = linspace(simResults.(char(taskName)).(modelName{mm}).muscleActivation.time(1),...
% % %                 simResults.(char(taskName)).(modelName{mm}).muscleActivation.time(end),101)';        
% % %             %Loop through activations while placing on appropriate subplot
% % %             for aa = 1:length(actNames)
% % %                 %Create normalised version of the data
% % %                 muscDatNorm = interp1(simResults.(char(taskName)).(modelName{mm}).muscleActivation.time,...
% % %                     simResults.(char(taskName)).(modelName{mm}).muscleActivation.(actNames{aa}),tNorm);
% % %                 subplot(5,6,subplotPos(aa)); hold on
% % %                 plot(0:100,muscDatNorm,'Color',lineColour{mm},'LineWidth',1.5)
% % %                 %Cleanup
% % %                 clear muscDatNorm
% % %             end
% % %             clear aa tNorm
% % %         end
% % %         clear mm
% % % 
% % %         %Set axes parameters
% % %         for aa = 1:length(subplotPos)
% % %             subplot(5,6,subplotPos(aa));
% % %             %Set font characteristics
% % %             set(gca,'FontSize',9,'FontWeight','bold','FontName','Helvetica');
% % %             %Set x axis label
% % %             xlabel('0-100% Task Completion',...
% % %                 'FontWeight','bold','FontName','Helvetica',...
% % %                 'FontSize',9);
% % %             %Set y axis label
% % %             label = [actNames{aa},' Activation (0-1)'];
% % %             ylabel(label,...
% % %                 'FontWeight','bold','FontName','Helvetica',...
% % %                 'FontSize',9);    
% % %             clear label
% % %             %Set axes style
% % %             ax = gca; box on; ax.LineWidth = 1;
% % %             set(gca,'Layer','top');
% % %             %Set axes limits
% % %             ax.XLim = [0 100];
% % %             ax.YLim = [0 1];
% % %             clear ax
% % %         end
% % %         clear aa
% % % 
% % %         %Add legend using custom legendflex function
% % %         ax = subplot(5,6,subplotPos(end-3));
% % %         legendflex(ax, modelLeg, 'anchor', {'s','n'}, 'buffer', [0 -45], 'nrow', length(modelLeg));
% % %         clear ax
% % % 
% % %         %Save figure
% % %         print(['MuscleActivationsNormScale_',(char(taskName)),'_fig.eps'],'-depsc2');        %eps format
% % %         set(gcf, 'PaperPositionMode','auto')
% % %         saveas(gcf,['MuscleActivationsNormScale_',(char(taskName)),'_fig.png']);             %low res png
% % %         saveas(gcf,['MuscleActivationsNormScale_',(char(taskName)),'_fig.fig']);             %matlab figure
% % %         print(gcf,['MuscleActivationsNormScale_',(char(taskName)),'_fig'],'-dtiff','-r600'); %600 dpi tif
% % %         %Close
% % %         close all
% % % 
% % %         %% Muscle cost
% % % 
% % % % % %         %Set a labelling variable for plot
% % % % % %         muscleGroupsLabels = [{'Scapula Protractors'};
% % % % % %             {'External Rotators'};
% % % % % %             {'Scapula Retractors'};
% % % % % %             {'Abductors'};
% % % % % %             {'Internal Rotators'};
% % % % % %             {'Flexors'};
% % % % % %             {'Horizontal Adductors'}];
% % % 
% % %         %All muscles
% % % 
% % %         figure; hold on
% % %         
% % %         %Plot normalised muscle cost values for all models (except for None)
% % %         for mm = 2:length(modelName)
% % %             b = bar(mm-1,simResults.(char(taskName)).(modelName{mm}).muscleCostTotalNorm.all);
% % %             b.FaceColor = lineColour{mm}; b.EdgeColor = [0,0,0]; b.BarWidth = 1; b.LineWidth = 1;
% % %         end
% % %         clear mm
% % % 
% % %         %Set formatting on axes numbers
% % %         set(gca,'FontSize',10,'FontWeight','bold','FontName','Helvetica');
% % %         ax = gca; box on; ax.LineWidth = 1;
% % %         set(gca,'Layer','top');
% % %         %Set axes limits and labels
% % %         ax.XLim = [0.3 8.7];
% % %         for mm = 2:length(modelName)
% % %             ax.XTickLabel{mm-1} = modelLeg{mm};
% % %         end
% % %         ax.XTickLabelRotation = 45;
% % %         ylabel('% \Delta in Muscle Cost',...
% % %             'FontWeight','bold','FontName','Helvetica',...
% % %             'FontSize',12);
% % %                 
% % %         %Save figure
% % %         print(['MuscleCostAll_',(char(taskName)),'_fig.eps'],'-depsc2');        %eps format
% % %         set(gcf, 'PaperPositionMode','auto')
% % %         saveas(gcf,['MuscleCostAll_',(char(taskName)),'_fig.png']);             %low res png
% % %         saveas(gcf,['MuscleCostAll_',(char(taskName)),'_fig.fig']);             %matlab figure
% % %         print(gcf,['MuscleCostAll_',(char(taskName)),'_fig'],'-dtiff','-r600'); %600 dpi tif
% % %         %Close
% % %         close all
% % %         
% % %         %Individual muscles
% % %         
% % %         figure; hold on
% % %         %Get current figure position and set size for the subplot
% % %         f = gcf; figPos = f.Position;
% % %         f.Position = [figPos(1)-figPos(4)*1.25 figPos(2)-figPos(3) figPos(3)*3 figPos(4)*2.5];
% % %         clear f figPos
% % %         
% % %         %Loop through models and plot
% % %         for mm = 2:length(modelName)   
% % %             %Loop through muscles while placing on appropriate subplot
% % %             for aa = 1:length(actNames)
% % %                 subplot(5,6,subplotPos(aa)); hold on
% % %                 b = bar(mm-1,simResults.(char(taskName)).(modelName{mm}).muscleCostIntNorm.(actNames{aa}));
% % %                 b.FaceColor = lineColour{mm}; b.EdgeColor = [0,0,0]; b.BarWidth = 1; b.LineWidth = 1;
% % %             end
% % %             clear aa
% % %         end
% % %         clear mm
% % %         
% % %         %Set axes parameters
% % %         for aa = 1:length(subplotPos)
% % %             subplot(5,6,subplotPos(aa));
% % %             %Set axes style
% % %             ax = gca; box on; ax.LineWidth = 1;
% % %             set(gca,'Layer','top');
% % %             %Set axes limits and labels
% % %             ax.XLim = [0.2 8.8];
% % %             %Set font characteristics
% % %             set(gca,'FontSize',9,'FontWeight','bold','FontName','Helvetica');
% % %             %Set title for figure
% % %             title(actNames{aa});
% % %             %Set x tick parameters (unlabeled)
% % %             ax.XTick = 1:1:8; ax.XTickLabel = {};
% % %             %Set y axis label
% % %             ylabel('% \Delta in Muscle Cost',...
% % %                 'FontWeight','bold','FontName','Helvetica','FontSize',9);    
% % %             clear ax
% % %         end
% % %         clear aa
% % % 
% % %         %Add legend using custom legendflex function
% % %         ax = subplot(5,6,subplotPos(end-3));
% % %         legendflex(ax, modelLeg(2:end), 'anchor', {'s','n'}, 'buffer', [0 -50], 'nrow', length(modelLeg(2:end)));
% % %         clear ax
% % % 
% % %         %Save figure
% % %         print(['MuscleCostIndividual_',(char(taskName)),'_fig.eps'],'-depsc2');        %eps format
% % %         set(gcf, 'PaperPositionMode','auto')
% % %         saveas(gcf,['MuscleCostIndividual_',(char(taskName)),'_fig.png']);             %low res png
% % %         saveas(gcf,['MuscleCostIndividual_',(char(taskName)),'_fig.fig']);             %matlab figure
% % %         print(gcf,['MuscleCostIndividual_',(char(taskName)),'_fig'],'-dtiff','-r600'); %600 dpi tif
% % %         %Close
% % %         close all
% % %         
% % %         %Individual muscles scaled to total involvement in all muscle cost
% % %         %i.e. relative contribution to the change in muscle cost
% % %         
% % %         figure; hold on
% % %         %Get current figure position and set size for the subplot
% % %         f = gcf; figPos = f.Position;
% % %         f.Position = [figPos(1)-figPos(4)*1.25 figPos(2)-figPos(3) figPos(3)*3 figPos(4)*2.5];
% % %         clear f figPos
% % %         
% % %         %Loop through models and plot
% % %         for mm = 2:length(modelName)   
% % %             %Loop through muscles while placing on appropriate subplot
% % %             for aa = 1:length(actNames)
% % %                 %Calculate this muscles relative contribution to the change
% % %                 %in overall muscle cost
% % %                 %Get the change in total muscle cost
% % %                 changeCost = simResults.(char(taskName)).(modelName{mm}).muscleCostTotal.all - ...
% % %                     simResults.(char(taskName)).None.muscleCostTotal.all;
% % %                 %Calculate the change in this muscles cost
% % %                 changeMusc = simResults.(char(taskName)).(modelName{mm}).muscleCostInt.(actNames{aa}) - ...
% % %                     simResults.(char(taskName)).None.muscleCostInt.(actNames{aa});
% % %                 %Calculate the relative change
% % %                 relCost = abs(changeMusc) / changeCost * 100;
% % %                 subplot(5,6,subplotPos(aa)); hold on
% % %                 b = bar(mm-1,relCost); b.FaceColor = lineColour{mm}; b.EdgeColor = [0,0,0]; b.BarWidth = 1; b.LineWidth = 1;
% % %                 %Cleanup
% % %                 clear changeCost changeMusc relCost
% % %             end
% % %             clear aa
% % %         end
% % %         clear mm
% % %         
% % %         %Loop through subplots and find the absolute max/min value to scale
% % %         %axes by
% % %         currScale = 0;
% % %         for aa = 1:length(subplotPos)
% % %             %Get axes
% % %             subplot(5,6,subplotPos(aa)); ax = gca;
% % %             %Get max and min y limits
% % %             yLims = [abs(ax.YLim(1));abs(ax.YLim(2))];
% % %             %Get max of limits
% % %             yLims = max(yLims);
% % %             %If larger than curretn scale, reset the value
% % %             if yLims > currScale
% % %                 currScale = yLims;
% % %             end
% % %             %Cleanup
% % %             clear ax yLims
% % %         end
% % %         clear aa
% % %         
% % %         %Set axes parameters
% % %         for aa = 1:length(subplotPos)
% % %             subplot(5,6,subplotPos(aa));
% % %             %Set axes style
% % %             ax = gca; box on; ax.LineWidth = 1;
% % %             set(gca,'Layer','top');
% % %             %Set axes limits and labels
% % %             ax.XLim = [0.2 8.8];
% % %             if ax.YLim(1) < 0
% % %                 ax.YLim(1) = currScale*-1;
% % %             end
% % %             if ax.YLim(2) > 0
% % %                 ax.YLim(2) = currScale;                
% % %             end
% % %             %Set font characteristics
% % %             set(gca,'FontSize',9,'FontWeight','bold','FontName','Helvetica');
% % %             %Set title for figure
% % %             title(actNames{aa});
% % %             %Set x tick parameters (unlabeled)
% % %             ax.XTick = 1:1:8; ax.XTickLabel = {};
% % %             %Set y axis label
% % %             ylabel('% Contrib. to \Delta Muscle Cost',...
% % %                 'FontWeight','bold','FontName','Helvetica','FontSize',8);    
% % %             clear ax
% % %         end
% % %         clear aa
% % % 
% % %         %Add legend using custom legendflex function
% % %         ax = subplot(5,6,subplotPos(end-3));
% % %         legendflex(ax, modelLeg(2:end), 'anchor', {'s','n'}, 'buffer', [0 -50], 'nrow', length(modelLeg(2:end)));
% % %         clear ax
% % % 
% % %         %Save figure
% % %         print(['MuscleCostIndividualRelative_',(char(taskName)),'_fig.eps'],'-depsc2');        %eps format
% % %         set(gcf, 'PaperPositionMode','auto')
% % %         saveas(gcf,['MuscleCostIndividualRelative_',(char(taskName)),'_fig.png']);             %low res png
% % %         saveas(gcf,['MuscleCostIndividualRelative_',(char(taskName)),'_fig.fig']);             %matlab figure
% % %         print(gcf,['MuscleCostIndividualRelative_',(char(taskName)),'_fig'],'-dtiff','-r600'); %600 dpi tif
% % %         %Close
% % %         close all

% % %         %Individual muscles scaled to total involvement in all muscle cost
% % %         %scaled to muscle cost
% % %         %i.e. relative contribution to the change in muscle cost MULTIPLIED BY
% % %         %the total percentage change in muscle cost
% % %         
% % %         figure; hold on
% % %         %Get current figure position and set size for the subplot
% % %         f = gcf; figPos = f.Position;
% % %         f.Position = [figPos(1)-figPos(4)*1.25 figPos(2)-figPos(3) figPos(3)*3 figPos(4)*2.5];
% % %         clear f figPos
% % %         
% % %         %Loop through models and plot
% % %         for mm = 2:length(modelName)   
% % %             %Loop through muscles while placing on appropriate subplot
% % %             for aa = 1:length(actNames)
% % %                 %Calculate this muscles relative contribution to the change
% % %                 %in overall muscle cost
% % %                 %Get the change in total muscle cost
% % %                 changeCost = simResults.(char(taskName)).(modelName{mm}).muscleCostTotal.all - ...
% % %                     simResults.(char(taskName)).None.muscleCostTotal.all;
% % %                 %Calculate the change in this muscles cost
% % %                 changeMusc = simResults.(char(taskName)).(modelName{mm}).muscleCostInt.(actNames{aa}) - ...
% % %                     simResults.(char(taskName)).None.muscleCostInt.(actNames{aa});
% % %                 %Calculate the relative change including total muscle cost
% % %                 relCost = changeMusc / changeCost * (simResults.(char(taskName)).(modelName{mm}).muscleCostTotalNorm.all/100) * 100;
% % %                 subplot(5,6,subplotPos(aa)); hold on
% % %                 b = bar(mm-1,relCost); b.FaceColor = lineColour{mm}; b.EdgeColor = [0,0,0]; b.BarWidth = 1; b.LineWidth = 1;
% % %                 %Cleanup
% % %                 clear changeCost changeMusc relCost
% % %             end
% % %             clear aa
% % %         end
% % %         clear mm
% % %         
% % %         %Loop through subplots and find the absolute max/min value to scale
% % %         %axes by
% % %         currScale = 0;
% % %         for aa = 1:length(subplotPos)
% % %             %Get axes
% % %             subplot(5,6,subplotPos(aa)); ax = gca;
% % %             %Get max and min y limits
% % %             yLims = [abs(ax.YLim(1));abs(ax.YLim(2))];
% % %             %Get max of limits
% % %             yLims = max(yLims);
% % %             %If larger than curretn scale, reset the value
% % %             if yLims > currScale
% % %                 currScale = yLims;
% % %             end
% % %             %Cleanup
% % %             clear ax yLims
% % %         end
% % %         clear aa
% % %         
% % %         %Round up scale
% % %         currScale = ceil(currScale);
% % %         
% % %         %Set axes parameters
% % %         for aa = 1:length(subplotPos)
% % %             subplot(5,6,subplotPos(aa));
% % %             %Set axes style
% % %             ax = gca; box on; ax.LineWidth = 1;
% % %             set(gca,'Layer','top');
% % %             %Set axes limits and labels
% % %             ax.XLim = [0.2 8.8];
% % %             if ax.YLim(1) < 0
% % %                 ax.YLim(1) = currScale*-1;
% % %             end
% % %             if ax.YLim(2) > 0
% % %                 ax.YLim(2) = currScale;                
% % %             end
% % %             %Set font characteristics
% % %             set(gca,'FontSize',9,'FontWeight','bold','FontName','Helvetica');
% % %             %Set title for figure
% % %             title(actNames{aa});
% % %             %Set x tick parameters (unlabeled)
% % %             ax.XTick = 1:1:8; ax.XTickLabel = {};
% % %             %Set y axis label
% % %             
% % %             %%%%%%TODO: fix y axes label/units for this figure
% % %             
% % %             ylabel('% Contrib. to \Delta Muscle Cost',...
% % %                 'FontWeight','bold','FontName','Helvetica','FontSize',8);    
% % %             clear ax
% % %         end
% % %         clear aa
% % % 
% % %         %Add legend using custom legendflex function
% % %         ax = subplot(5,6,subplotPos(end-3));
% % %         legendflex(ax, modelLeg(2:end), 'anchor', {'s','n'}, 'buffer', [0 -50], 'nrow', length(modelLeg(2:end)));
% % %         clear ax
% % % 
% % %         %Save figure
% % %         print(['MuscleCostIndividualRelativeTotal_',(char(taskName)),'_fig.eps'],'-depsc2');        %eps format
% % %         set(gcf, 'PaperPositionMode','auto')
% % %         saveas(gcf,['MuscleCostIndividualRelativeTotal_',(char(taskName)),'_fig.png']);             %low res png
% % %         saveas(gcf,['MuscleCostIndividualRelativeTotal_',(char(taskName)),'_fig.fig']);             %matlab figure
% % %         print(gcf,['MuscleCostIndividualRelativeTotal_',(char(taskName)),'_fig'],'-dtiff','-r600'); %600 dpi tif
% % %         %Close
% % %         close all


% % %         
% % %         
% % %         %%%%% TO DO: output data for tables...
% % %         
        %Navigate back to results directory
        cd(resultsDir);
        
    end
    clear tt

% % % %% Create mean figures
% % % 
% % % mkdir('MeanResults'); cd('MeanResults');
% % % mkdir('Figures'); cd('Figures');
% % % 
% % % % Muscle cost
% % % 
%Calculate mean and standard deviation for muscle cost
%Start at 2nd model
% % % for mm = 2:length(modelName)
% % %    for tt = 1:length(simTasks)
% % %        %Extract values to temporary dataset
% % %        X(mm-1,tt) = simResults.(simTasks{tt}).(modelName{mm}).muscleCostTotalNorm.all;       
% % %    end
% % %    clear tt
% % %    %Calculate mean and standard deviation
% % %    costM(1,mm-1) = mean(X(mm-1,:)); costSD(1,mm-1) = std(X(mm-1,:));
% % % end
% % % clear mm
% % % 
% % % %Create mean & SD plot for muscle cost
% % % figure; hold on
% % % 
% % % %Plot the mean and standard deviation
% % % for dd = 1:length(costSD)
% % %     %Do standard deviation first so it sits behind
% % %     lineSD = line([dd;dd],[costM(dd)-costSD(dd);costM(dd)+costSD(dd)],...
% % %         'Color','k','LineWidth',2);
% % %     %Plot the mean as an individual scatter
% % %     dotM = scatter(dd,costM(dd),300);
% % %     dotM.Marker = 'square'; dotM.LineWidth = 1.5;
% % %     dotM.MarkerEdgeColor = 'k'; dotM.MarkerFaceColor = lineColour{dd+1};
% % %     %Plot the individual values next to it
% % %     indD = scatter([dd;dd;dd],X(dd,:),15);
% % %     indD.Marker = 'o'; indD.LineWidth = 1;
% % %     indD.MarkerEdgeColor = 'k'; indD.MarkerFaceColor = 'w';
% % % end
% % % clear dd
% % % 
% % % %Set formatting on axes numbers
% % % set(gca,'FontSize',10,'FontWeight','bold','FontName','Helvetica');
% % % ax = gca; box on; ax.LineWidth = 1;
% % % set(gca,'Layer','top');
% % % %Set axes limits and labels
% % % ax.XLim = [0.3 8.7];
% % % for mm = 2:length(modelName)
% % %     ax.XTickLabel{mm-1} = modelLeg{mm};
% % % end
% % % ax.XTickLabelRotation = 45;
% % % ylabel('% \Delta in Muscle Cost',...
% % %     'FontWeight','bold','FontName','Helvetica',...
% % %     'FontSize',12);
% % % 
% % % %Save and close
% % % print('MuscleCostAll_Mean_fig.eps','-depsc2');        %eps format
% % % set(gcf, 'PaperPositionMode','auto')
% % % saveas(gcf,'MuscleCostAll_Mean_fig.png');             %low res png
% % % saveas(gcf,'MuscleCostAll_Mean_fig.fig');             %matlab figure
% % % print(gcf,'MuscleCostAll_Mean_fig','-dtiff','-r600'); %600 dpi tif
% % % %Close
% % % close all
% % % 
% % % %Cleanup
% % % clear X costM costSD lineSD dotM indD ax

%% Calculate error statistics

% Calculate mean absolute error of shoulder joint angles relative to the 'None' model

for tt = 1:length(simTasks)
    
    %Calculate the time normalised values for each shoulder angle (None model)
    %Time scale
    tNorm = linspace(simResults.(simTasks{tt}).None.coordinateValue.time(1),...
        simResults.(simTasks{tt}).None.coordinateValue.time(end),101)';
    %Interpolate data
    elvDatNorm = interp1(simResults.(simTasks{tt}).None.coordinateValue.time,...
        simResults.(simTasks{tt}).None.coordinateValue.shoulder_elv,tNorm);
    angDatNorm = interp1(simResults.(simTasks{tt}).None.coordinateValue.time,...
        simResults.(simTasks{tt}).None.coordinateValue.elv_angle,tNorm);
    rotDatNorm = interp1(simResults.(simTasks{tt}).None.coordinateValue.time,...
        simResults.(simTasks{tt}).None.coordinateValue.shoulder_rot,tNorm);
    
    %Loop through remaining models and calculate mean absolute error
    for mm = 2:length(modelName)
        
        %Create time normalised versions of data for current model
        %Time scale
        tNormX = linspace(simResults.(simTasks{tt}).(modelName{mm}).coordinateValue.time(1),...
            simResults.(simTasks{tt}).(modelName{mm}).coordinateValue.time(end),101)';
        %Interpolate data
        elvDatNormX = interp1(simResults.(simTasks{tt}).(modelName{mm}).coordinateValue.time,...
            simResults.(simTasks{tt}).(modelName{mm}).coordinateValue.shoulder_elv,tNormX);
        angDatNormX = interp1(simResults.(simTasks{tt}).(modelName{mm}).coordinateValue.time,...
            simResults.(simTasks{tt}).(modelName{mm}).coordinateValue.elv_angle,tNormX);
        rotDatNormX = interp1(simResults.(simTasks{tt}).(modelName{mm}).coordinateValue.time,...
            simResults.(simTasks{tt}).(modelName{mm}).coordinateValue.shoulder_rot,tNormX);
        
        %Calculatethe absolute error at each time step and average
        %Convert to degrees from radians in process
        meanAbsoluteError.(simTasks{tt}).(modelName{mm}).coordinateValue.shoulder_elv = ...
            mean(abs(rad2deg(elvDatNormX) - rad2deg(elvDatNorm)));
        meanAbsoluteError.(simTasks{tt}).(modelName{mm}).coordinateValue.elv_angle = ...
            mean(abs(rad2deg(angDatNormX) - rad2deg(angDatNorm)));
        meanAbsoluteError.(simTasks{tt}).(modelName{mm}).coordinateValue.shoulder_rot = ...
            mean(abs(rad2deg(rotDatNormX) - rad2deg(rotDatNorm)));
        
        %Cleanup
        clear tNormX elvDatNormX angDatNormX rotDatNormX        
        
    end
    clear mm
    
    %Cleanup
    clear tNorm elvDatNorm angDatNorm rotDatNorm
    
end
clear tt

% % % %Place these error values in a temporary matrix for transfer to paper table
% % % for mm = 2:length(modelName)
% % %     %Upward reach
% % %     tempTbl(mm-1,1) = meanAbsoluteError.(simTasks{1}).(modelName{mm}).coordinateValue.shoulder_elv;
% % %     tempTbl(mm-1,2) = meanAbsoluteError.(simTasks{1}).(modelName{mm}).coordinateValue.elv_angle;
% % %     tempTbl(mm-1,3) = meanAbsoluteError.(simTasks{1}).(modelName{mm}).coordinateValue.shoulder_rot;
% % %     %Forward reach
% % %     tempTbl(mm-1,4) = meanAbsoluteError.(simTasks{2}).(modelName{mm}).coordinateValue.shoulder_elv;
% % %     tempTbl(mm-1,5) = meanAbsoluteError.(simTasks{2}).(modelName{mm}).coordinateValue.elv_angle;
% % %     tempTbl(mm-1,6) = meanAbsoluteError.(simTasks{2}).(modelName{mm}).coordinateValue.shoulder_rot;
% % %     %Hair touch
% % %     tempTbl(mm-1,7) = meanAbsoluteError.(simTasks{3}).(modelName{mm}).coordinateValue.shoulder_elv;
% % %     tempTbl(mm-1,8) = meanAbsoluteError.(simTasks{3}).(modelName{mm}).coordinateValue.elv_angle;
% % %     tempTbl(mm-1,9) = meanAbsoluteError.(simTasks{3}).(modelName{mm}).coordinateValue.shoulder_rot;
% % % end
% % % clear mm
% % % % % % clear tempTbl

% % % % Create a temporary matrix for the performance times to transfer to paper
% % % for mm = 1:length(modelName)
% % %     %Upward reach
% % %     tempTbl(mm,1) = simResults.(simTasks{1}).(modelName{mm}).perfTime;
% % %     tempTbl(mm,2) = simResults.(simTasks{2}).(modelName{mm}).perfTime;
% % %     tempTbl(mm,3) = simResults.(simTasks{3}).(modelName{mm}).perfTime;
% % % end
% % % clear mm
% % % % % % clear tempTbl

%% Create a combined tiled figure for total muscle cost

%Load in figures
cd([simTasks{1},'\Figures']);
task1 = hgload(['MuscleCostAll_',simTasks{1},'_fig.fig']);
% close all;
cd('..\..');
cd([simTasks{2},'\Figures']);
task2 = hgload(['MuscleCostAll_',simTasks{2},'_fig.fig']);
% close all;
cd('..\..');
cd([simTasks{3},'\Figures']);
task3 = hgload(['MuscleCostAll_',simTasks{3},'_fig.fig']);
% close all;
cd('..\..');
cd('MeanResults\Figures');
taskAll = hgload('MuscleCostAll_Mean_fig.fig');
% close all;

%Initialise figure
figure

% Get current figure position and set size for the subplot
f = gcf; figPos = f.Position;
f.Position = [figPos(1)-figPos(4)*1.25 figPos(2)-figPos(3) figPos(3)*3 figPos(4)*2];
clear f figPos

%Prepare subplots
h(1) = subplot(2,3,2);
h(2) = subplot(2,3,4); h(3) = subplot(2,3,5); h(4) = subplot(2,3,6);

%Paste figures onto axes

%All mean data
copyobj(allchild(get(taskAll,'CurrentAxes')),h(1));
subplot(2,3,2);
set(gca,'FontSize',10,'FontWeight','bold','FontName','Helvetica');
ax = gca; box on; ax.LineWidth = 1;
set(gca,'Layer','top');
%Set axes limits and labels
ax.XLim = [0.3 8.7];
for mm = 2:length(modelName)
    ax.XTickLabel{mm-1} = modelLeg{mm};
end
ax.XTickLabelRotation = 45;
ylabel('% \Delta in Muscle Cost',...
    'FontWeight','bold','FontName','Helvetica',...
    'FontSize',12);
%Reset size for this axes (1.25 scale)
ax.Position = [ax.Position(1)-(ax.Position(3)*0.125) ax.Position(2)-(ax.Position(4)*0.05) ax.Position(3)*1.25 ax.Position(4)*1.25]; 
%Place letter label in corner
ylim = get(gca,'ylim'); xlim = get(gca,'xlim');
text(xlim(1)+xlim(2)*0.025,ylim(2)-ylim(2)*0.075,'A',...
    'FontSize',20,'FontWeight','bold','FontName','Helvetica');

%remaining subplots
for pp = 4:6
    %Copy object
    if pp == 4
        copyobj(allchild(get(task1,'CurrentAxes')),h(2));
        subplot(2,3,4);
        plotText = 'B';
    elseif pp == 5
        copyobj(allchild(get(task2,'CurrentAxes')),h(3));
        subplot(2,3,5);
        plotText = 'C';
    else
        copyobj(allchild(get(task3,'CurrentAxes')),h(4));
        subplot(2,3,6);
        plotText = 'D';
    end
    %Set formatting on axes numbers
    set(gca,'FontSize',10,'FontWeight','bold','FontName','Helvetica');
    ax = gca; box on; ax.LineWidth = 1;
    set(gca,'Layer','top');
    %Set axes limits and labels
    ax.XLim = [0.3 8.7];
    for mm = 2:length(modelName)
        ax.XTickLabel{mm-1} = modelLeg{mm};
    end
    ax.XTickLabelRotation = 45;
    ylabel('% \Delta in Muscle Cost',...
        'FontWeight','bold','FontName','Helvetica',...
        'FontSize',12);
    %Place letter label in corner
    if pp == 5
        ylim = get(gca,'ylim'); xlim = get(gca,'xlim');
        text(xlim(1)+xlim(2)*0.025,ylim(2)-ylim(2)*0.125,plotText,...
            'FontSize',20,'FontWeight','bold','FontName','Helvetica');
    else
        ylim = get(gca,'ylim'); xlim = get(gca,'xlim');
        text(xlim(1)+xlim(2)*0.025,ylim(2)-ylim(2)*0.075,plotText,...
            'FontSize',20,'FontWeight','bold','FontName','Helvetica');
    end
end
clear pp

%Save and close
print('MuscleCostAll_Combined_fig.eps','-depsc2');        %eps format
set(gcf, 'PaperPositionMode','auto')
saveas(gcf,'MuscleCostAll_Combined_fig.png');             %low res png
saveas(gcf,'MuscleCostAll_Combined_fig.fig');             %matlab figure
print(gcf,'MuscleCostAll_Combined_fig','-dtiff','-r600'); %600 dpi tif
%Close
close all



close all
cd('..\..');

end
