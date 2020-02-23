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


    %% Extract simulation results
    
    %Navigate to results path
    cd(['..\..\SimulationResults\',taskName]);

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
    
    %% Loop through files and extract data
    for mm = 1:length(modelName)
        
        %Grab the solution related to the current model
        fileInd = logical(contains(taskFiles,modelName{mm}));
        fileName = taskFiles(fileInd);
        currSolution = MocoTrajectory(fileName{1,1});
        clear fileInd fileName
        
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
        
        %% Calculate muscle variables
        
        %%%%%%% TO DO: add muscle related varaible calculations...
        
    end
    clear mm

    %% Plot results
    
    %%%%%% TO DO: worthwhile plotting absolute values with respect to time,
    %%%%%% but may be better to present time normalised data???
    
    %Create figures directory
    mkdir('Figures'); cd('Figures');
    
    %Set colour scheme variables for the nine conditions
    %%%%%%%% TO DO: check this colour scheme -- update earlier figures with
    %%%%%%%% new colour scheme
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
    
    %%% Performance time value %%%
    
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
    %Set legend with custom function
    
    %%%%%%% TO DO: need to adjust figure size to support this
    
% % %     legendflex(ax, modelLeg, 'anchor', {'s','n'}, 'buffer', [0 0], 'nrow', 3);
    
    

%%%%%%%%%%%%%%%%%%%%%%%%% below plots are fine but performance time isn't
%%%%%%%%%%%%%%%%%%%%%%%%% finished...

    
    %%% Shoulder joint angles %%%
    
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
    print(['ShoulderAngles_',(char(taskName)),'_ModelComparison_fig.eps'],'-depsc2');        %eps format
    set(gcf, 'PaperPositionMode','auto')
    saveas(gcf,['ShoulderAngles_',(char(taskName)),'_ModelComparison_fig.png']);             %low res png
    saveas(gcf,['ShoulderAngles_',(char(taskName)),'_ModelComparison_fig.fig']);             %matlab figure
    print(gcf,['ShoulderAngles_',(char(taskName)),'_ModelComparison_fig'],'-dtiff','-r600'); %600 dpi tif
    %Close
    close all
    %Cleanup
    clear maxTime

    %%% Shoulder joint speeds %%%
    
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
            rad2deg(simResults.(char(taskName)).(modelName{mm}).coordinateSpeed.shoulder_elv),...
            'Color',lineColour{mm},'LineWidth',1.5)        
        %Elevation plane
        subplot(1,3,2); hold on
        plot(simResults.(char(taskName)).(modelName{mm}).coordinateValue.time,...
            rad2deg(simResults.(char(taskName)).(modelName{mm}).coordinateSpeed.elv_angle),...
            'Color',lineColour{mm},'LineWidth',1.5)        
        %Shoulder rotation
        subplot(1,3,3); hold on
        plot(simResults.(char(taskName)).(modelName{mm}).coordinateValue.time,...
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
        %%%%% NOTE: legends don't place great for speeds, and they don't
        %%%%% look great either...
        if aa == 3
            legend(modelLeg,'Location','SouthEast'); legend boxoff
        else
            legend(modelLeg,'Location','NorthWest'); legend boxoff
        end
    end
    clear aa

    %Save figure
    print(['ShoulderVelocities_',(char(taskName)),'_ModelComparison_fig.eps'],'-depsc2');        %eps format
    set(gcf, 'PaperPositionMode','auto')
    saveas(gcf,['ShoulderVelocities_',(char(taskName)),'_ModelComparison_fig.png']);             %low res png
    saveas(gcf,['ShoulderVelocities_',(char(taskName)),'_ModelComparison_fig.fig']);             %matlab figure
    print(gcf,['ShoulderVelocities_',(char(taskName)),'_ModelComparison_fig'],'-dtiff','-r600'); %600 dpi tif
    %Close
    close all
    %Cleanup
    clear maxTime
    
    %%% Muscle activations %%%
    
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
    print(['MuscleActivations_',(char(taskName)),'_ModelComparison_fig.eps'],'-depsc2');        %eps format
    set(gcf, 'PaperPositionMode','auto')
    saveas(gcf,['MuscleActivations_',(char(taskName)),'_ModelComparison_fig.png']);             %low res png
    saveas(gcf,['MuscleActivations_',(char(taskName)),'_ModelComparison_fig.fig']);             %matlab figure
    print(gcf,['MuscleActivations_',(char(taskName)),'_ModelComparison_fig'],'-dtiff','-r600'); %600 dpi tif
    %Close
    close all
    %Cleanup
    clear maxTime


%%

end
