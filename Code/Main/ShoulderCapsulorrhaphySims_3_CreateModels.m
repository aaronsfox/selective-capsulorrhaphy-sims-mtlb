function ShoulderCapsulorrhaphySims_3_CreateModels

% @author: Aaron Fox
% Centre for Sport Research, Deakin University
% aaron.f@deakin.edu.au
% 
% This code takes the parameters from the previous scripts optimisation and
% adds the equations into the custom dual expression based force class for 
% the different capsulorrhaphy conditions simulated.
%
% This code also creates visualisations of the force applied by these
% 'ligaments' across the different motions and positions for the different
% capsulorrhaphy conditions simulated. Along with the visualisations, the 
% peak angles achieved across the different capsulorrhaphy conditions are 
% calculated and compared to the Gerber et al. study values.
%
% TO DO: add Gerber et al. references

    %% Set-up
	
	import org.opensim.modeling.*

    %Set main directory
    mainDir = pwd;

    %Add supplementary code folder to path
    addpath(genpath('..\Supplementary'));
	
	%Load in the Gerber et al. database for joint angles, and angles and
    %plication labels
    %This is created by running the 'createGerber2003_anglesDatabase' function
    %from the supplementary code folder
    cd('..\..\SupportingData');
    load('Gerber2003_AnglesDatabase.mat','angles','meanAngles','plications');
	
	%Navigate to model directory
    cd('..\ModelFiles');
    modelDir = pwd;
    %Add geometry directory
    ModelVisualizer.addDirToGeometrySearchPaths([pwd,'\Geometry']);
	
	%Load the dual expression based force plugin library
	cd('..\Plugin_DualEBCF\build\Release');
    opensimCommon.LoadOpenSimLibraryExact([pwd,'\osimDualEBCF.dll']);	
	
	%Load in the optimisation results
    cd('..\..\..\ModelFiles\OptimisationResults');
	load('OptimisedLigamentParameters.mat','coordForceFunc','ligParamOpt','optResults');
    
    %% Create basic versions of capsulorrhaphy models
    
    %Navigate to model directory
    cd('..');
    
    %Classes developed from plugins are difficult to work in Matlab with
    %using the API, so the workaround to this is to work with the model
    %files as text or XML-bsaed files.
    
    %Read in the basic shoulder complex with the blank dual expression
    %based coordinate force to fill with data from the optimisation.
    [baseModelXML, RootName, ~] = xml_readOSIM('BasicShoulderComplex_withBlankDualEBCF.osim');
    
    %Set preferences for writing to avoid itemisation of forces
    Pref.StructItem = false;
    
    %Create damping forces for the model coordinates to add in each time
    
    %Shoulder elevation damping CLF
    elvDamp = CoordinateLimitForce();            %constructor with default properties
    elvDamp.setName('shoulder_elv_damping');     %set name
    elvDamp.set_coordinate('shoulder_elv');      %set coordinate
    elvDamp.set_upper_stiffness(1e-08);          %set upper stiffness value
    elvDamp.set_upper_limit(190);                %set upper limit threshold
    elvDamp.set_lower_stiffness(1e-08);          %set lower stiffness value
    elvDamp.set_lower_limit(190);                %set lower limit threshold
    elvDamp.set_damping(0.001745);               %set damping value
    elvDamp.set_transition(1);                   %set transition value
        
    %Shoulder rotation damping CLF
    rotDamp = CoordinateLimitForce();            %constructor with default properties
    rotDamp.setName('shoulder_rot_damping');     %set name
    rotDamp.set_coordinate('shoulder_rot');      %set coordinate
    rotDamp.set_upper_stiffness(1e-08);          %set upper stiffness value
    rotDamp.set_upper_limit(190);                %set upper limit threshold
    rotDamp.set_lower_stiffness(1e-08);          %set lower stiffness value
    rotDamp.set_lower_limit(190);                %set lower limit threshold
    rotDamp.set_damping(0.001745);               %set damping value
    rotDamp.set_transition(1);                   %set transition value
    
    %Set the parameters for the two coordinate forces in each model
    for pp = 1:length(plications)

        %Create a copy of the base model to edit
        copyModelXML = baseModelXML;

        %Shoulder elevation (only need positive expression for this one)
        copyModelXML.Model.ForceSet.objects.DualExpressionBasedCoordinateForce(1).coordinate = 'shoulder_elv';
        copyModelXML.Model.ForceSet.objects.DualExpressionBasedCoordinateForce(1).coordinateDual = 'elv_angle';
        copyModelXML.Model.ForceSet.objects.DualExpressionBasedCoordinateForce(1).expressionPos = coordForceFunc.(plications{pp}).Elv;

        %Shoulder rotation
        copyModelXML.Model.ForceSet.objects.DualExpressionBasedCoordinateForce(2).coordinate = 'shoulder_rot';
        copyModelXML.Model.ForceSet.objects.DualExpressionBasedCoordinateForce(2).coordinateDual = 'shoulder_elv';
        copyModelXML.Model.ForceSet.objects.DualExpressionBasedCoordinateForce(2).expressionPos = coordForceFunc.(plications{pp}).IntRot;
        copyModelXML.Model.ForceSet.objects.DualExpressionBasedCoordinateForce(2).expressionNeg = coordForceFunc.(plications{pp}).ExtRot;

        %Rename model
        copyModelXML.Model.ATTRIBUTE.name = ['BasicCapsModel_',plications{pp}];
        
        %Rename the expression force objects
        copyModelXML.Model.ForceSet.objects.DualExpressionBasedCoordinateForce(1).ATTRIBUTE.name = 'shoulder_elv_DualEBCF';
        copyModelXML.Model.ForceSet.objects.DualExpressionBasedCoordinateForce(2).ATTRIBUTE.name = 'shoulder_rot_DualEBCF';

        %Export edited .osim file from XML
        xml_writeOSIM(['BasicCapsModel_',plications{pp},'.osim'],copyModelXML,RootName,Pref);
        
        %Bring the model back in to add the damping force for ligaments
        editModel.(plications{pp}) = Model(['BasicCapsModel_',plications{pp},'.osim']);
        
        %Add the damping forces to model
        editModel.(plications{pp}).addForce(elvDamp)
        editModel.(plications{pp}).addForce(rotDamp)
        
        %Finalise connections
        editModel.(plications{pp}).finalizeConnections();
        
        %Re-save the model
        editModel.(plications{pp}).print(['BasicCapsModel_',plications{pp},'.osim']);
        
        %Cleanup
        clear copyModelXML
        
    end
    clear pp
    
    %% Run simulations with each model similar to the optimisation procedure
    %  to ensure that the desired end joint positions are being achieved by
    %  each capsulorrhaphy model
    
    %Create variables that set the starting/locked joint angles for the
    %different simulation positions. This variable is order in columns of:
    %shoulder elevation, elevation angle and shoulder rotation
    startingAngles = [0, 30, 0;
        0, 90, 0;
        0, 30, 0;
        0, 30, 0;
        45, 30, 0;
        45, 30, 0;
        90, 30, 0;
        90, 30, 0];
    
    coordLock = [false, true, true;
        false, true, true;
        true, true, false;
        true, true, false;
        true, true, false;
        true, true, false;
        true, true, false;
        true, true, false];
    
    %Set an empty variable to store the end joint angles and errors in
    expAngles = zeros(length(plications),length(angles));
    
    %Create a directory to store optimised results in
    mkdir('OptimisedResults');
    
    %Loop through the different angles and models to run forward simulations
    for pp = 1:length(plications)
        for aa = 1:length(angles)
            
            %%%%% NOTE: below bits commented out to avoid running forward
            %%%%% analyses every time this code is tested
            
% % %             %Load in the current model
% % %             osimModel = Model(['BasicCapsModel_',plications{pp},'.osim']);
            
            %Set the motion being examined
            motion = angles{aa};
% % %             
% % %             %Set the default coordinate values for the starting positions
% % %             %based on the motion being tested
% % %             osimModel.getCoordinateSet().get('shoulder_elv').setDefaultValue(deg2rad(startingAngles(aa,1)));
% % %             osimModel.getCoordinateSet().get('elv_angle').setDefaultValue(deg2rad(startingAngles(aa,2)));
% % %             osimModel.getCoordinateSet().get('shoulder_rot').setDefaultValue(deg2rad(startingAngles(aa,3)));
% % %             
% % %             %Lock the relevant coordintes based on the motion being tested
% % %             osimModel.getCoordinateSet().get('shoulder_elv').set_locked(coordLock(aa,1));
% % %             osimModel.getCoordinateSet().get('elv_angle').set_locked(coordLock(aa,2));
% % %             osimModel.getCoordinateSet().get('shoulder_rot').set_locked(coordLock(aa,3));
% % %             
% % %             %Add a force reporter analysis to the model for the forward sim
% % %             FrAnalysis = ForceReporter();
% % %             if contains(motion,'Rot')
% % %                 FrAnalysis.setStartTime(0); FrAnalysis.setEndTime(3);
% % %             else
% % %                 FrAnalysis.setStartTime(0); FrAnalysis.setEndTime(5);
% % %             end
% % %             osimModel.getAnalysisSet().cloneAndAppend(FrAnalysis);
% % %             
% % %             %Finalise model connections
% % %             osimModel.finalizeConnections();
% % %             
% % %             %Initialise the forward tool
% % %             FwdTool = ForwardTool();
% % % 
% % %             %Settings for forward tool
% % %             FwdTool.setModel(osimModel);
% % % 
% % %             %Set times based on motion. Rotations = 3 seconds; Elevations = 5 seconds
% % %             if contains(motion,'Rot')
% % %                 FwdTool.setInitialTime(0); FwdTool.setFinalTime(3);
% % %             else
% % %                 FwdTool.setInitialTime(0); FwdTool.setFinalTime(5);
% % %             end
% % %             
% % %             %Set controls for simulation, depending on whether it is an elevation or
% % %             %internal/external rotation motion
% % %             if contains(motion,'IntRot')
% % %                 FwdTool.setControlsFileName([modelDir,'\shoulder_introt_controls.xml']);
% % %             elseif contains(motion,'ExtRot')
% % %                 FwdTool.setControlsFileName([modelDir,'\shoulder_extrot_controls.xml']);
% % %             else
% % %                 FwdTool.setControlsFileName([modelDir,'\shoulder_elv_controls.xml']);
% % %             end
% % %             
% % %             %Need to add controller set for the controls to be active in the model
% % %             %First, clear any existing controller set from past simulations
% % %             osimModel.getControllerSet().clearAndDestroy();
% % %             %Add the current controller set to the model
% % %             FwdTool.addControllerSetToModel();
% % %             
% % %             %Rename the forward tool based on parameters
% % %             FwdTool.setName(motion);
            
            %Navigate to results folder
            cd('OptimisedResults');
% % %             
% % %             %If on first run through for model, create directory for its results
% % %             if aa == 1
% % %                 mkdir(plications{pp})
% % %             end
            
            %Navigate to plication directory
            cd(plications{pp});
% % % 
% % %             %Run forward tool
% % %             FwdTool.run();

            %Load in states and force data
            statesD = importdata([motion,'_states_degrees.mot']);
            forceD = importdata([motion,'_ForceReporter_forces.sto']);

            %Grab out the relevant shoulder angle and force data
            if contains(motion,'Rot')
                angInd = contains(statesD.colheaders,'shoulder_rot') & contains(statesD.colheaders,'value');
                forInd = strcmp(forceD.colheaders,'shoulder_rot_DualEBCF');
            else
                angInd = contains(statesD.colheaders,'shoulder_elv') & contains(statesD.colheaders,'value');
                forInd = strcmp(forceD.colheaders,'shoulder_elv_DualEBCF');
            end
            
            %Get and store the shoulder angle and force data
            simResults.(plications{pp}).(angles{aa}).shoulderAngle = statesD.data(:,angInd);
            simResults.(plications{pp}).(angles{aa}).ligForce = forceD.data(:,forInd);
            
            %Calculate and store peak angles and errors
            if contains(motion,'IntRot')
                expAngles(pp,aa) = max(simResults.(plications{pp}).(angles{aa}).shoulderAngle);                
            elseif contains(motion,'ExtRot')
                expAngles(pp,aa) = min(simResults.(plications{pp}).(angles{aa}).shoulderAngle);
            else
                expAngles(pp,aa) = max(simResults.(plications{pp}).(angles{aa}).shoulderAngle);                
            end
            
            %Cleanup
            clear angInd forInd statesD forceD osimModel motion FrAnalysis FwdTool
            
            %Navigate back to base directory
            cd('..\..');

        end
        clear aa
    end
    clear pp
    
    %Calculate difference between Gerber et al. angles and experimental angles
    errAngles = expAngles - meanAngles;
    pErrAngles = errAngles ./ meanAngles * 100;
    
    %Convert error data to table format and write to text file
    cd('OptimisedResults');
    %Convert to table
    expAnglesT = array2table(expAngles,'VariableNames',angles,'RowNames',plications); 
    errAnglesT = array2table(errAngles,'VariableNames',angles,'RowNames',plications);
    pErrAnglesT = array2table(pErrAngles,'VariableNames',angles,'RowNames',plications);
    %Write to file (tab  delimited)
    writetable(expAnglesT,'ExperimentalFinalAngles.txt','Delimiter','\t','WriteRowNames',true);
    writetable(errAnglesT,'AbsoluteErrorsFinalAngles.txt','Delimiter','\t','WriteRowNames',true);
    writetable(pErrAnglesT,'PercentageErrorsFinalAngles.txt','Delimiter','\t','WriteRowNames',true);
    
    cd('..');
    
    %% Create plots of 'ligament' forces vs. shoulder angles
    
    %These plots will be visualised across a relevant spectrum of joint
    %angles, rather than what is achieved during the simulations - and
    %therefore will be calculated from the coordinate functions
    
    %First convert strings to function handles
    for pp = 1:length(plications)
        coordForceFunc.(plications{pp}).f_IntRot = ...
            str2func(append('@(q1,q2)',convertCharsToStrings(coordForceFunc.(plications{pp}).IntRot)));
        coordForceFunc.(plications{pp}).f_ExtRot = ...
            str2func(append('@(q1,q2)',convertCharsToStrings(coordForceFunc.(plications{pp}).ExtRot)));
        coordForceFunc.(plications{pp}).f_Elv = ...
            str2func(append('@(q1,q2)',convertCharsToStrings(coordForceFunc.(plications{pp}).Elv)));
    end
    clear pp
    
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
    
    %Create new text cases for figure legends
    plicationsLegend = [{'None'};
        {'Anterosuperior'};
        {'Anteroinferior'};
        {'Total Anterior'};
        {'Posterosuperior'};
        {'Posteroinferior'};
        {'Total Posterior'};
        {'Total Superior'};
        {'Total Inferior'}];
    
    %Set q2 positions for each figure
    q2Val = [30,90,0,0,45,45,90,90];
    
    %Create and navigate to figure directory
    mkdir('OptimisedFigures'); cd('OptimisedFigures');
    
    %%%%% Create a subplot for the elevation figures %%%%%
    
    %Initialise figure
    figure; hold on
    
    %Get current figure position and double length for the subplot
    f = gcf; figPos = f.Position;
    f.Position = [figPos(1) figPos(2)-figPos(4) figPos(3) figPos(4)*2];
    clear f figPos
    
    %Loop through angles
    for aa = [1,2]  %elevation motions

        %Initialise subplot 
        subplot(2,1,aa);
        hold on
        
        %Loop through plications
        for pp = 1:length(plications)

            %Create variables for q1 and q2 inputs of function
            %Take the maximum angle that is achieved for current motion
            q1 = deg2rad(linspace(0,expAngles(pp,aa),100));
            q2 = deg2rad(ones(1,100)*q2Val(aa));

            %Evaluate function at each step
            f = coordForceFunc.(plications{pp}).f_Elv;            
            y = zeros(length(q1),1);
            for tt = 1:length(q1)
                y(tt) = feval(f,q1(tt),q2(tt));
            end

            %Plot force against shoulder angle 
            %Invert as forces for elevation produce negative values
            plot(rad2deg(q1),y*-1,'Color',lineColour{pp,1},'LineWidth',1.5);

            %Cleanup
            clear y q1 q2 f 

        end
        clear pp

        %Set x-axis limits to peak angle reached
        set(gca,'xlim',([0,max(expAngles(:,aa))]));

        %Set axes labels
        xlabel(['Elevation Angle (',char(176),')'],...
            'FontWeight','bold','FontName','Helvetica',...
            'FontSize',12);
        ylabel('Force (N)',...
            'FontWeight','bold','FontName','Helvetica',...
            'FontSize',12);

        %Set formatting on axes numbers
        set(gca,'FontSize',10,'FontWeight','bold','FontName','Helvetica');

        %Set framing of figure
        ax = gca; box on; ax.LineWidth = 1; clear ax
        set(gca,'Layer','top');
    
    end
    clear aa
    
    %Check which axes has the larger x-value and set both to this
    subplot(2,1,1); ax = gca; xMax1 = ax.XLim(2); clear ax
    subplot(2,1,2); ax = gca; xMax2 = ax.XLim(2); clear ax
    %Fix appropriate axes
    if xMax1 < xMax2
        subplot(2,1,1); ax = gca; ax.XLim(2) = xMax2; clear ax
    elseif xMax2 < xMax1
        subplot(2,1,2); ax = gca; ax.XLim(2) = xMax1; clear ax
    end
    %Cleanup
    clear xMax1 xMax2
    
    %Set appropriate labels in top left corner of figure
    %Top axis
    subplot(2,1,1); ylim = get(gca,'ylim'); xlim = get(gca,'xlim');
    text(xlim(1)+xlim(2)*0.025,ylim(2)-ylim(2)*0.075,'A',...
        'FontSize',20,'FontWeight','bold','FontName','Helvetica');
    subplot(2,1,2); ylim = get(gca,'ylim'); xlim = get(gca,'xlim');
    text(xlim(1)+xlim(2)*0.025,ylim(2)-ylim(2)*0.075,'B',...
        'FontSize',20,'FontWeight','bold','FontName','Helvetica');
    
    %Add legend using custom legendflex function
    ax = subplot(2,1,2);
    legendflex(ax, plicationsLegend, 'anchor', {'s','n'}, 'buffer', [0 -70], 'nrow', 3);
    clear ax
    
    %Cleanup subplot positioning
    %Get positions of each axis
    ha = get(gcf,'children');
    h1pos = get(ha(2),'position');  %bottom axis
    h2pos = get(ha(3),'position');  %top axis
    %Tighten up bottom subplot to top one
    set(ha(2),'position',[h1pos(1) h2pos(2)-h2pos(4)-0.075 h1pos(3) h1pos(4)]);
        
    %Save and print figure options
    print('LigamentForces_Elevation_fig.eps','-depsc2');        %eps format
    set(gcf, 'PaperPositionMode','auto')
    saveas(gcf,'LigamentForces_Elevation_fig.png');             %low res png
    saveas(gcf,'LigamentForces_Elevation_fig.fig');             %matlab figure
    print(gcf,'LigamentForces_Elevation_fig','-dtiff','-r600'); %600 dpi tif
    
    %Cleanup and close
    clear h1pos h2pos ha xlim ylim
    close all
    
    %%%%% Create a subplot for the rotation figures %%%%%
    
    %Initialise figure
    figure; hold on
    
    %Get current figure position and triple length and double width for the subplot
    f = gcf; figPos = f.Position;
    f.Position = [figPos(1)-figPos(3)*0.5 figPos(2)-figPos(4)*1.2 figPos(3)*2 figPos(4)*3];
    clear f figPos
    
    %Loop through angles
    for aa = 3:length(angles)  %elevation motions

        %Initialise subplot 
        subplot(3,2,aa-2);
        hold on
        
        %Loop through plications
        for pp = 1:length(plications)

            %Create variables for q1 and q2 inputs of function
            %Take the maximum angle that is achieved for current motion
            q1 = deg2rad(linspace(0,expAngles(pp,aa),100));
            q2 = deg2rad(ones(1,100)*q2Val(aa));

            %Evaluate function at each step
            %Set function based on motion being tested
            if contains(angles{aa},'IntRot')
                f = coordForceFunc.(plications{pp}).f_IntRot;
            elseif contains(angles{aa},'ExtRot')
                f = coordForceFunc.(plications{pp}).f_ExtRot;
            end            
            y = zeros(length(q1),1);
            for tt = 1:length(q1)
                y(tt) = feval(f,q1(tt),q2(tt));
            end

            %Plot force against shoulder angle 
            %Invert as forces for internal rotation as the produce negative values.
            %Also invert external rotation angles
            %as they are negative.
            if contains(angles{aa},'ExtRot')
                plot(rad2deg(q1)*-1,y,'Color',lineColour{pp,1},'LineWidth',1.5);
            else
                plot(rad2deg(q1),y*-1,'Color',lineColour{pp,1},'LineWidth',1.5);
            end

            %Cleanup
            clear y q1 q2 f 

        end
        clear pp

        %Set x-axis limits to peak angle reached
        %Put a check in place to invert external rotation angles
        if contains(angles{aa},'ExtRot')
            set(gca,'xlim',([0,max(expAngles(:,aa)*-1)]));
        else
            set(gca,'xlim',([0,max(expAngles(:,aa))]));
        end

        %Set axes labels
        if contains(angles{aa},'IntRot')
            xlabel(['Internal Rotation Angle (',char(176),')'],...
                'FontWeight','bold','FontName','Helvetica',...
                'FontSize',10);
        elseif contains(angles{aa},'ExtRot')
            xlabel(['External Rotation Angle (',char(176),')'],...
                'FontWeight','bold','FontName','Helvetica',...
                'FontSize',10);
        end
        ylabel('Force (N)',...
            'FontWeight','bold','FontName','Helvetica',...
            'FontSize',10);

        %Set formatting on axes numbers
        set(gca,'FontSize',9,'FontWeight','bold','FontName','Helvetica');

        %Set framing of figure
        ax = gca; box on; ax.LineWidth = 1; clear ax
        set(gca,'Layer','top');
    
    end
    clear aa
    
    %Check which axes has the largest x-value and set all to this
    %External rotation
    subplot(3,2,1); ax = gca; xMax1 = ax.XLim(2); clear ax
    subplot(3,2,3); ax = gca; xMax2 = ax.XLim(2); clear ax
    subplot(3,2,5); ax = gca; xMax3 = ax.XLim(2); clear ax
    %Fix appropriate axes
    if xMax1 > xMax2 && xMax1 > xMax3
        subplot(3,2,3); ax = gca; ax.XLim(2) = xMax1; clear ax
        subplot(3,2,5); ax = gca; ax.XLim(2) = xMax1; clear ax
    elseif xMax2 > xMax1 && xMax2 > xMax3
        subplot(3,2,1); ax = gca; ax.XLim(2) = xMax2; clear ax
        subplot(3,2,5); ax = gca; ax.XLim(2) = xMax2; clear ax
    elseif xMax3 > xMax1 && xMax3 > xMax2
        subplot(3,2,1); ax = gca; ax.XLim(2) = xMax2; clear ax
        subplot(3,2,3); ax = gca; ax.XLim(2) = xMax2; clear ax
    end
    %Cleanup
    clear xMax1 xMax2 xMax3
    %Internal rotation
    subplot(3,2,2); ax = gca; xMax1 = ax.XLim(2); clear ax
    subplot(3,2,4); ax = gca; xMax2 = ax.XLim(2); clear ax
    subplot(3,2,6); ax = gca; xMax3 = ax.XLim(2); clear ax
    %Fix appropriate axes
    if xMax1 > xMax2 && xMax1 > xMax3
        subplot(3,2,4); ax = gca; ax.XLim(2) = xMax1; clear ax
        subplot(3,2,6); ax = gca; ax.XLim(2) = xMax1; clear ax
    elseif xMax2 > xMax1 && xMax2 > xMax3
        subplot(3,2,2); ax = gca; ax.XLim(2) = xMax2; clear ax
        subplot(3,2,6); ax = gca; ax.XLim(2) = xMax2; clear ax
    elseif xMax3 > xMax1 && xMax3 > xMax2
        subplot(3,2,2); ax = gca; ax.XLim(2) = xMax2; clear ax
        subplot(3,2,4); ax = gca; ax.XLim(2) = xMax2; clear ax
    end
    %Cleanup
    clear xMax1 xMax2 xMax3
    
    %Set appropriate labels in top left corner of figure
    %Top axis
    subplot(3,2,1); ylim = get(gca,'ylim'); xlim = get(gca,'xlim');
    text(xlim(1)+xlim(2)*0.025,ylim(2)-ylim(2)*0.075,'A',...
        'FontSize',18,'FontWeight','bold','FontName','Helvetica');
    subplot(3,2,3); ylim = get(gca,'ylim'); xlim = get(gca,'xlim');
    text(xlim(1)+xlim(2)*0.025,ylim(2)-ylim(2)*0.075,'B',...
        'FontSize',18,'FontWeight','bold','FontName','Helvetica');
    subplot(3,2,5); ylim = get(gca,'ylim'); xlim = get(gca,'xlim');
    text(xlim(1)+xlim(2)*0.025,ylim(2)-ylim(2)*0.075,'C',...
        'FontSize',18,'FontWeight','bold','FontName','Helvetica');
    subplot(3,2,2); ylim = get(gca,'ylim'); xlim = get(gca,'xlim');
    text(xlim(1)+xlim(2)*0.025,ylim(2)-ylim(2)*0.075,'D',...
        'FontSize',18,'FontWeight','bold','FontName','Helvetica');
    subplot(3,2,4); ylim = get(gca,'ylim'); xlim = get(gca,'xlim');
    text(xlim(1)+xlim(2)*0.025,ylim(2)-ylim(2)*0.075,'E',...
        'FontSize',18,'FontWeight','bold','FontName','Helvetica');
    subplot(3,2,6); ylim = get(gca,'ylim'); xlim = get(gca,'xlim');
    text(xlim(1)+xlim(2)*0.025,ylim(2)-ylim(2)*0.075,'F',...
        'FontSize',18,'FontWeight','bold','FontName','Helvetica');
    
    %Add legend using custom legendflex function
    ax = subplot(3,2,6);
    legendflex(ax, plicationsLegend, 'anchor', {'s','n'}, 'buffer', [-250 -50], 'nrow', 3);
    clear ax
    
    %Cleanup subplot positioning
    %Get positions of each axis
    ha = get(gcf,'children');
    h1pos = get(ha(2),'position');  %bottom right axis
    h2pos = get(ha(3),'position');  %bottom left axis
    h3pos = get(ha(4),'position');  %middle right axis
    h4pos = get(ha(5),'position');  %middle left axis
    h5pos = get(ha(6),'position');  %top right axis
    h6pos = get(ha(7),'position');  %top left axis
    %Tighten up bottom subplot to top one
    set(ha(5),'position',[h4pos(1) h6pos(2)-h6pos(4)-0.06 h4pos(3) h4pos(4)]);
    set(ha(4),'position',[h3pos(1) h5pos(2)-h5pos(4)-0.06 h3pos(3) h3pos(4)]);
    %Reset middle axes positions
    h3pos = get(ha(4),'position');  %middle right axis
    h4pos = get(ha(5),'position');  %middle left axis
    set(ha(2),'position',[h1pos(1) h3pos(2)-h3pos(4)-0.06 h1pos(3) h1pos(4)]);
    set(ha(3),'position',[h2pos(1) h4pos(2)-h4pos(4)-0.06 h2pos(3) h2pos(4)]);
        
    %Save and print figure options
    print('LigamentForces_Rotation_fig.eps','-depsc2');        %eps format
    set(gcf, 'PaperPositionMode','auto')
    saveas(gcf,'LigamentForces_Rotation_fig.png');             %low res png
    saveas(gcf,'LigamentForces_Rotation_fig.fig');             %matlab figure
    print(gcf,'LigamentForces_Rotation_fig','-dtiff','-r600'); %600 dpi tif
    
    %Cleanup and close
    clear h1pos h2pos h3pos h4pos h5pos h6pos ha xlim ylim
    close all
    
    %% Create full shoulder models with appropriate 'ligament' forces
    
    cd('..');
    
    %Loop through plications and copy relevant coordinate forces into a
    %base model of the full shoulder. This is also a good opportunity to
    %convert the model muscles to the DeGroote muscle format that is used
    %within Moco
    
    %Load in base model
    baseModel = Model('FullShoulderModel.osim');
    
    %Convert Millard muscles in model to DeGroote format
    DeGrooteFregly2016Muscle.replaceMuscles(baseModel);
    
    %Loop through plications
    for pp = 1:length(plications)
        
        %Set the model name
        baseModel.setName(['FullShoulderModel_',plications{pp}]);
        
        %Load in the appropriate basic model with the coordinate forces
        forceModel = Model(['BasicCapsModel_',plications{pp},'.osim']);
        
        %Get the appropriate coordinate forces from the basic shoulder
        %model and append them to the current copy of the full shoulder
        %model. This includes the dual expression based coordinate forces
        %and the damping coordinate limit forces. If it is on more than the
        %first iteration these forces first need to be deleted
        if pp > 1
            %First delete the original force entries added on earlier iterations
            baseModel.getForceSet().remove(baseModel.getForceSet().get('shoulder_elv_DualEBCF'));
            baseModel.getForceSet().remove(baseModel.getForceSet().get('shoulder_rot_DualEBCF'));
            baseModel.getForceSet().remove(baseModel.getForceSet().get('shoulder_elv_damping'));
            baseModel.getForceSet().remove(baseModel.getForceSet().get('shoulder_rot_damping'));
            %Now add back in from force model
            baseModel.getForceSet().cloneAndAppend(forceModel.getForceSet().get('shoulder_elv_DualEBCF'));
            baseModel.getForceSet().cloneAndAppend(forceModel.getForceSet().get('shoulder_rot_DualEBCF'));
            baseModel.getForceSet().cloneAndAppend(forceModel.getForceSet().get('shoulder_elv_damping'));
            baseModel.getForceSet().cloneAndAppend(forceModel.getForceSet().get('shoulder_rot_damping'));
        else
            baseModel.getForceSet().cloneAndAppend(forceModel.getForceSet().get('shoulder_elv_DualEBCF'));
            baseModel.getForceSet().cloneAndAppend(forceModel.getForceSet().get('shoulder_rot_DualEBCF'));
            baseModel.getForceSet().cloneAndAppend(forceModel.getForceSet().get('shoulder_elv_damping'));
            baseModel.getForceSet().cloneAndAppend(forceModel.getForceSet().get('shoulder_rot_damping'));
        end
        
        %Finalise connections
        baseModel.finalizeConnections();
        
        %Save new model
        baseModel.print(['FullShoulderModel_',plications{pp},'.osim']);
        
    end
    clear pp
    
%----- End of ShoulderCapsulorrhaphySims_3_CreateModels.m -----%