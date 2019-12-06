function totalErr = ligamentOptimiser(x,Input)

% @author: Aaron Fox
% Centre for Sport Research, Deakin University
% aaron.f@deakin.edu.au
% 
% Script serves as the optimisation cost function within the minimsation
% search function.
% 
% TO DO: Add extra notes if necessary...

import org.opensim.modeling.*

%% Set-up

%Get model
global osimModel

% % % %Reload plugin
% % % opensimCommon.LoadOpenSimLibraryExact(Input.pluginPath);

%Extract inputs
plication = Input.plicationType;
motion = Input.motionType;
modelDir = Input.modelDir;

%Set the ligament parameters
switch motion
    
    case 'Elevation'
        %CLF forces
        CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_elv_ligaments')).set_upper_limit(x(1));
        CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_elv_ligaments')).set_upper_stiffness(x(2));
        CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_elv_ligaments')).set_transition(x(3));
        %Custom CLF's
        %Requires the current osimModel to be printed and imported as XML
        %to access the custom class properties
        osimModel.print('tempModel.osim');
        [osimXML, RootName, ~] = xml_readOSIM('tempModel.osim');
        %Find relevant elevation by elevation plane ligament
        for c = 1:length(osimXML.Model.ForceSet.objects.CustomCoordinateLimitForce)
            if strcmp(osimXML.Model.ForceSet.objects.CustomCoordinateLimitForce(c).ATTRIBUTE.name,'shoulder_elv_by_elv_angle_ligaments')
                clfInd = c;
            else
            end
        end
        clear c
        %Set the properties
        osimXML.Model.ForceSet.objects.CustomCoordinateLimitForce(clfInd).upper_limit = x(4);
        osimXML.Model.ForceSet.objects.CustomCoordinateLimitForce(clfInd).lower_limit = x(5);
        osimXML.Model.ForceSet.objects.CustomCoordinateLimitForce(clfInd).upper_stiffness = x(6);
        osimXML.Model.ForceSet.objects.CustomCoordinateLimitForce(clfInd).transition = x(7);
        %Reprint updated model
        DOMnode = xml_writeOSIM('updatedModel.osim',osimXML,RootName);
        %Reload in updated model to global variable
        osimModel = Model('updatedModel.osim');
        %Cleanup added files and variables
        delete tempModel.osim updatedModel.osim
        clear osimXML RootName DOMnode
    
    case 'IntRot0'        
        %CLF forces
        CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).set_upper_limit(x(1));
        CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).set_upper_stiffness(x(2));
        CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).set_transition(x(3));
% % %         CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).set_lower_stiffness(x(4));
% % %         CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).set_transition(x(5));
% % %         %Custom CLF's
% % %         %Requires the current osimModel to be printed and imported as XML
% % %         %to access the custom class properties
% % %         osimModel.print('tempModel.osim');
% % %         [osimXML, RootName, ~] = xml_readOSIM('tempModel.osim');
% % %         %Find relevant elevation by elevation plane ligament
% % %         for c = 1:length(osimXML.Model.ForceSet.objects.CustomCoordinateLimitForce)
% % %             if strcmp(osimXML.Model.ForceSet.objects.CustomCoordinateLimitForce(c).ATTRIBUTE.name,'shoulder_rot_by_shoulder_elv_ligaments')
% % %                 clfInd = c;
% % %             else
% % %             end
% % %         end
% % %         clear c
% % %         %Set the properties
% % %         osimXML.Model.ForceSet.objects.CustomCoordinateLimitForce(clfInd).upper_limit = x(6);
% % %         osimXML.Model.ForceSet.objects.CustomCoordinateLimitForce(clfInd).upper_stiffness = x(7);
% % %         osimXML.Model.ForceSet.objects.CustomCoordinateLimitForce(clfInd).transition = x(8);
% % %         %Reprint updated model
% % %         DOMnode = xml_writeOSIM('updatedModel.osim',osimXML,RootName);
% % %         %Reload in updated model to global variable
% % %         osimModel = Model('updatedModel.osim');
% % %         %Cleanup added files and variables
% % %         delete tempModel.osim updatedModel.osim
% % %         clear osimXML RootName DOMnode

    case 'IntRot45'     %%%%% TO DO: this really wouldn't need to be done for each different IR version     
        %CLF forces
        CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).set_upper_limit(x(1));
        CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).set_upper_stiffness(x(2));
        CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).set_transition(x(3));
% % %         CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).set_lower_stiffness(x(4));
% % %         CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).set_transition(x(5));
% % %         %Custom CLF's
% % %         %Requires the current osimModel to be printed and imported as XML
% % %         %to access the custom class properties
% % %         osimModel.print('tempModel.osim');
% % %         [osimXML, RootName, ~] = xml_readOSIM('tempModel.osim');
% % %         %Find relevant elevation by elevation plane ligament
% % %         for c = 1:length(osimXML.Model.ForceSet.objects.CustomCoordinateLimitForce)
% % %             if strcmp(osimXML.Model.ForceSet.objects.CustomCoordinateLimitForce(c).ATTRIBUTE.name,'shoulder_rot_by_shoulder_elv_ligaments')
% % %                 clfInd = c;
% % %             else
% % %             end
% % %         end
% % %         clear c
% % %         %Set the properties
% % %         osimXML.Model.ForceSet.objects.CustomCoordinateLimitForce(clfInd).upper_limit = x(6);
% % %         osimXML.Model.ForceSet.objects.CustomCoordinateLimitForce(clfInd).upper_stiffness = x(7);
% % %         osimXML.Model.ForceSet.objects.CustomCoordinateLimitForce(clfInd).transition = x(8);
% % %         %Reprint updated model
% % %         DOMnode = xml_writeOSIM('updatedModel.osim',osimXML,RootName);
% % %         %Reload in updated model to global variable
% % %         osimModel = Model('updatedModel.osim');
% % %         %Cleanup added files and variables
% % %         delete tempModel.osim updatedModel.osim
% % %         clear osimXML RootName DOMnode

    case 'IntRot90'     %%%%% TO DO: this really wouldn't need to be done for each different IR version     
        %CLF forces
        CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).set_upper_limit(x(1));
        CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).set_upper_stiffness(x(2));
        CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).set_transition(x(3));
% % %         CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).set_lower_stiffness(x(4));
% % %         CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).set_transition(x(5));
% % %         %Custom CLF's
% % %         %Requires the current osimModel to be printed and imported as XML
% % %         %to access the custom class properties
% % %         osimModel.print('tempModel.osim');
% % %         [osimXML, RootName, ~] = xml_readOSIM('tempModel.osim');
% % %         %Find relevant elevation by elevation plane ligament
% % %         for c = 1:length(osimXML.Model.ForceSet.objects.CustomCoordinateLimitForce)
% % %             if strcmp(osimXML.Model.ForceSet.objects.CustomCoordinateLimitForce(c).ATTRIBUTE.name,'shoulder_rot_by_shoulder_elv_ligaments')
% % %                 clfInd = c;
% % %             else
% % %             end
% % %         end
% % %         clear c
% % %         %Set the properties
% % %         osimXML.Model.ForceSet.objects.CustomCoordinateLimitForce(clfInd).upper_limit = x(6);
% % %         osimXML.Model.ForceSet.objects.CustomCoordinateLimitForce(clfInd).upper_stiffness = x(7);
% % %         osimXML.Model.ForceSet.objects.CustomCoordinateLimitForce(clfInd).transition = x(8);
% % %         %Reprint updated model
% % %         DOMnode = xml_writeOSIM('updatedModel.osim',osimXML,RootName);
% % %         %Reload in updated model to global variable
% % %         osimModel = Model('updatedModel.osim');
% % %         %Cleanup added files and variables
% % %         delete tempModel.osim updatedModel.osim
% % %         clear osimXML RootName DOMnode

    case 'RotationElevation'
        %Custom CLF's
        %Requires the current osimModel to be printed and imported as XML
        %to access the custom class properties
        osimModel.print('tempModel.osim');
        [osimXML, RootName, ~] = xml_readOSIM('tempModel.osim');
        %Find relevant elevation by elevation plane ligament
        for c = 1:length(osimXML.Model.ForceSet.objects.CustomCoordinateLimitForce)
            if strcmp(osimXML.Model.ForceSet.objects.CustomCoordinateLimitForce(c).ATTRIBUTE.name,'shoulder_rot_by_shoulder_elv_ligaments')
                clfInd = c;
            else
            end
        end
        clear c
        %Set the properties
        osimXML.Model.ForceSet.objects.CustomCoordinateLimitForce(clfInd).upper_limit = x(1);
        osimXML.Model.ForceSet.objects.CustomCoordinateLimitForce(clfInd).upper_stiffness = x(2);
        osimXML.Model.ForceSet.objects.CustomCoordinateLimitForce(clfInd).transition = x(3);
        %Reprint updated model
        DOMnode = xml_writeOSIM('updatedModel.osim',osimXML,RootName);
        %Reload in updated model to global variable
        osimModel = Model('updatedModel.osim');
        %Cleanup added files and variables
        delete tempModel.osim updatedModel.osim
        clear osimXML RootName DOMnode
        
end

%Set the desired joint angles based on movement and plication types

%%%%% TO DO: incorporate Gerber et al. database file into this process as
%%%%% an input matrix with the function

switch motion
    
    case 'Elevation'
        
        switch plication
            case 'None'
                desiredAbdAng = 91.5;
                desiredFlexAng = 85.6;
            %%%%% TO DO: add other plications
        end
        
    case 'IntRot0'  %%%%% TO DO: as above, probably doesn't need an iteration
        
        switch plication
            case 'None'
% % %                 desiredExtRotAng0 = 53.4;
                desiredIntRotAng0 = 44.6;
% % %                 desiredExtRotAng45 = 104.4;
% % %                 desiredIntRotAng45 = 39.0;
% % %                 desiredExtRotAng90 = 133.0;
% % %                 desiredIntRotAng90 = 30.8;
            %%%%% TO DO: add other plications
        end
        
    case 'IntRot45'
        
        switch plication
            case 'None'
% % %                 desiredExtRotAng0 = 53.4;
% % %                 desiredIntRotAng0 = 44.6;
% % %                 desiredExtRotAng45 = 104.4;
                desiredIntRotAng45 = 39.0;
% % %                 desiredExtRotAng90 = 133.0;
% % %                 desiredIntRotAng90 = 30.8;
            %%%%% TO DO: add other plications
        end
        
    case 'IntRot90'
        
        switch plication
            case 'None'
% % %                 desiredExtRotAng0 = 53.4;
% % %                 desiredIntRotAng0 = 44.6;
% % %                 desiredExtRotAng45 = 104.4;
% % %                 desiredIntRotAng45 = 39.0;
% % %                 desiredExtRotAng90 = 133.0;
                desiredIntRotAng90 = 30.8;
            %%%%% TO DO: add other plications
        end
        
    case 'RotationElevation'
        
        switch plication
            case 'None'
                desiredExtRotAng45 = 104.4;
                desiredIntRotAng45 = 39.0;
                desiredExtRotAng90 = 133.0;
                desiredIntRotAng90 = 30.8;
            %%%%% TO DO: add other plications
            
        end
end

%Set starting total error
totalErr = 0;

%% Run the abduction motion forward simulation if appropriate

if strcmp(motion,'Elevation')

    %Set the shoulder coordinate values
    osimModel.getCoordinateSet().get('elv_angle').setDefaultValue(deg2rad(30));
    osimModel.getCoordinateSet().get('shoulder_elv').setDefaultValue(deg2rad(0));
    osimModel.getCoordinateSet().get('shoulder_rot').setDefaultValue(deg2rad(0));

    %Set locking on shoulder coordinates
    osimModel.getCoordinateSet().get('elv_angle').set_locked(true);
    osimModel.getCoordinateSet().get('shoulder_elv').set_locked(false);
    osimModel.getCoordinateSet().get('shoulder_rot').set_locked(true);
    
    %Finalise model connections
    osimModel.finalizeConnections();

    %Initialise the forward tool
    FwdTool = ForwardTool();

    %Settings for forward tool
    FwdTool.setModel(osimModel);
    FwdTool.setInitialTime(0); FwdTool.setFinalTime(5);

    %Set controls for simulation
    FwdTool.setControlsFileName([modelDir,'\shoulder_elv_controls.xml']);
    
    %Need to add controller set for the controls to be active in the model
    %First, clear any existing controller set from past simulations
    osimModel.getControllerSet().clearAndDestroy();
    %Add the current controller set to the model
    FwdTool.addControllerSetToModel();

    %Run forward tool
    FwdTool.run();
    
    %Calculate errors
    
    %Load in states data
    D = importdata('_states_degrees.mot');
    
    %Grab out the shoulder elevation angle
    angInd = contains(D.colheaders,'shoulder_elv') & contains(D.colheaders,'value');
    elvAng = D.data(:,angInd);
    
    %Calculate the peak elevation angle
    peakElv = max(elvAng);
    
    %Identify absolute error between peak and desired angle and add to total error
    isoErr = abs(desiredAbdAng - peakElv);
    totalErr = totalErr + isoErr;
    
    %Cleanup
    clear D elvAng angInd peakElv isoErr
    
    %Clear forward simulation files
    delete _controls.sto _states.sto _states_degrees.mot
    
end

%% Run the flexion motion forward simulation if appropriate

if strcmp(motion,'Elevation')

    %Set the shoulder coordinate values
    osimModel.getCoordinateSet().get('elv_angle').setDefaultValue(deg2rad(90));
    osimModel.getCoordinateSet().get('shoulder_elv').setDefaultValue(deg2rad(0));
    osimModel.getCoordinateSet().get('shoulder_rot').setDefaultValue(deg2rad(0));

    %Set locking on shoulder coordinates
    osimModel.getCoordinateSet().get('elv_angle').set_locked(true);
    osimModel.getCoordinateSet().get('shoulder_elv').set_locked(false);
    osimModel.getCoordinateSet().get('shoulder_rot').set_locked(true);
    
    %Finalise model connections
    osimModel.finalizeConnections();

    %Initialise the forward tool
    FwdTool = ForwardTool();

    %Settings for forward tool
    FwdTool.setModel(osimModel);
    FwdTool.setInitialTime(0); FwdTool.setFinalTime(5);

    %Set controls for simulation
    FwdTool.setControlsFileName([modelDir,'\shoulder_elv_controls.xml']);
    
    %Need to add controller set for the controls to be active in the model
    %First, clear any existing controller set from past simulations
    osimModel.getControllerSet().clearAndDestroy();
    %Add the current controller set to the model
    FwdTool.addControllerSetToModel();

    %Run forward tool
    FwdTool.run();
    
    %Calculate errors
    
    %Load in states data
    D = importdata('_states_degrees.mot');
    
    %Grab out the shoulder elevation angle
    angInd = contains(D.colheaders,'shoulder_elv') & contains(D.colheaders,'value');
    elvAng = D.data(:,angInd);
    
    %Calculate the peak elevation angle
    peakElv = max(elvAng);
    
    %Identify absolute error between peak and desired angle and add to total error
    isoErr = abs(desiredFlexAng - peakElv);
    totalErr = totalErr + isoErr;
    
    %Cleanup
    clear D elvAng angInd peakElv isoErr
    
    %Clear forward simulation files
    delete _controls.sto _states.sto _states_degrees.mot
    
end

%% Run the external rotation at 0 degrees abduction forward simulation if appropriate

if strcmp(motion,'Rotation')

    %Set the shoulder coordinate values
    osimModel.getCoordinateSet().get('elv_angle').setDefaultValue(deg2rad(30));
    osimModel.getCoordinateSet().get('shoulder_elv').setDefaultValue(deg2rad(0));
    osimModel.getCoordinateSet().get('shoulder_rot').setDefaultValue(deg2rad(0));

    %Set locking on shoulder coordinates
    osimModel.getCoordinateSet().get('elv_angle').set_locked(true);
    osimModel.getCoordinateSet().get('shoulder_elv').set_locked(true);
    osimModel.getCoordinateSet().get('shoulder_rot').set_locked(false);
    
    %Finalise model connections
    osimModel.finalizeConnections();

    %Initialise the forward tool
    FwdTool = ForwardTool();

    %Settings for forward tool
    FwdTool.setModel(osimModel);
    FwdTool.setInitialTime(0); FwdTool.setFinalTime(3);

    %Set controls for simulation
    FwdTool.setControlsFileName([modelDir,'\shoulder_extrot_controls.xml']);
    
    %Need to add controller set for the controls to be active in the model
    %First, clear any existing controller set from past simulations
    osimModel.getControllerSet().clearAndDestroy();
    %Add the current controller set to the model
    FwdTool.addControllerSetToModel();

    %Run forward tool
    FwdTool.run();
    
    %Calculate errors
    
    %Load in states data
    D = importdata('_states_degrees.mot');
    
    %Grab out the shoulder rotation angle
    angInd = contains(D.colheaders,'shoulder_rot') & contains(D.colheaders,'value');
    rotAng = D.data(:,angInd);
    
    %Want to penalise the model if the rotation is occurring in the wrong
    %direction, so can first check if it is going in the appropriate
    %direction for the relevant rotation. Can check this by calculating the
    %mean of data and testing whether positive or negative.
    meanRot = mean(rotAng);
    
    if meanRot < 1
        %Calculate the peak external rotation angle
        peakRot = abs(min(rotAng));
    else
        %Calculate the peak internal rotation angle as negative
        peakRot = max(rotAng)*-1;
    end
    
    %Identify absolute error between peak and desired angle and add to total error
    isoErr = abs(desiredExtRotAng0 - peakRot);
    totalErr = totalErr + isoErr;
    
    %Cleanup
    clear D rotAng angInd peakRot isoErr
    
    %Clear forward simulation files
    delete _controls.sto _states.sto _states_degrees.mot
    
end

%% Run the internal rotation at 0 degrees abduction forward simulation if appropriate

if strcmp(motion,'IntRot0')

    %Set the shoulder coordinate values
    osimModel.getCoordinateSet().get('elv_angle').setDefaultValue(deg2rad(30));
    osimModel.getCoordinateSet().get('shoulder_elv').setDefaultValue(deg2rad(0));
    osimModel.getCoordinateSet().get('shoulder_rot').setDefaultValue(deg2rad(0));

    %Set locking on shoulder coordinates
    osimModel.getCoordinateSet().get('elv_angle').set_locked(true);
    osimModel.getCoordinateSet().get('shoulder_elv').set_locked(true);
    osimModel.getCoordinateSet().get('shoulder_rot').set_locked(false);
    
    %Finalise model connections
    osimModel.finalizeConnections();

    %Initialise the forward tool
    FwdTool = ForwardTool();

    %Settings for forward tool
    FwdTool.setModel(osimModel);
    FwdTool.setInitialTime(0); FwdTool.setFinalTime(3);

    %Set controls for simulation
    FwdTool.setControlsFileName([modelDir,'\shoulder_introt_controls.xml']);
    
    %Need to add controller set for the controls to be active in the model
    %First, clear any existing controller set from past simulations
    osimModel.getControllerSet().clearAndDestroy();
    %Add the current controller set to the model
    FwdTool.addControllerSetToModel();

    %Run forward tool
    FwdTool.run();
    
    %Calculate errors
    
    %Load in states data
    D = importdata('_states_degrees.mot');
    
    %Grab out the shoulder rotation angle
    angInd = contains(D.colheaders,'shoulder_rot') & contains(D.colheaders,'value');
    rotAng = D.data(:,angInd);
    
    %Want to penalise the model if the rotation is occurring in the wrong
    %direction, so can first check if it is going in the appropriate
    %direction for the relevant rotation. Can check this by calculating the
    %mean of data and testing whether positive or negative.
    meanRot = mean(rotAng);
    
    if meanRot > 1
        %Calculate the peak internal rotation angle
        peakRot = max(rotAng);
    else
        %Calculate the peak external rotation angle as negative
        peakRot = min(rotAng);
    end
    
    %Identify absolute error between peak and desired angle and add to total error
    isoErr = abs(desiredIntRotAng0 - peakRot);
    totalErr = totalErr + isoErr;
    
    %Cleanup
    clear D rotAng angInd peakRot isoErr
    
    %Clear forward simulation files
    delete _controls.sto _states.sto _states_degrees.mot
    
end

%% Run the external rotation at 45 degrees abduction forward simulation if appropriate

if strcmp(motion,'RotationElevation_ER')

    %Set the shoulder coordinate values
    osimModel.getCoordinateSet().get('elv_angle').setDefaultValue(deg2rad(30));
    osimModel.getCoordinateSet().get('shoulder_elv').setDefaultValue(deg2rad(45));
    osimModel.getCoordinateSet().get('shoulder_rot').setDefaultValue(deg2rad(0));

    %Set locking on shoulder coordinates
    osimModel.getCoordinateSet().get('elv_angle').set_locked(true);
    osimModel.getCoordinateSet().get('shoulder_elv').set_locked(true);
    osimModel.getCoordinateSet().get('shoulder_rot').set_locked(false);
    
    %Finalise model connections
    osimModel.finalizeConnections();

    %Initialise the forward tool
    FwdTool = ForwardTool();

    %Settings for forward tool
    FwdTool.setModel(osimModel);
    FwdTool.setInitialTime(0); FwdTool.setFinalTime(3);

    %Set controls for simulation
    FwdTool.setControlsFileName([modelDir,'\shoulder_extrot_controls.xml']);
    
    %Need to add controller set for the controls to be active in the model
    %First, clear any existing controller set from past simulations
    osimModel.getControllerSet().clearAndDestroy();
    %Add the current controller set to the model
    FwdTool.addControllerSetToModel();

    %Run forward tool
    FwdTool.run();
    
    %Calculate errors
    
    %Load in states data
    D = importdata('_states_degrees.mot');
    
    %Grab out the shoulder rotation angle
    angInd = contains(D.colheaders,'shoulder_rot') & contains(D.colheaders,'value');
    rotAng = D.data(:,angInd);
    
    %Want to penalise the model if the rotation is occurring in the wrong
    %direction, so can first check if it is going in the appropriate
    %direction for the relevant rotation. Can check this by calculating the
    %mean of data and testing whether positive or negative.
    meanRot = mean(rotAng);
    
    if meanRot < 1
        %Calculate the peak external rotation angle
        peakRot = abs(min(rotAng));
    else
        %Calculate the peak internal rotation angle as negative
        peakRot = max(rotAng)*-1;
    end
    
    %Identify absolute error between peak and desired angle and add to total error
    isoErr = abs(desiredExtRotAng45 - peakRot);
    totalErr = totalErr + isoErr;
    
    %Cleanup
    clear D rotAng angInd peakRot isoErr
    
    %Clear forward simulation files
    delete _controls.sto _states.sto _states_degrees.mot
    
end

%% Run the internal rotation at 45 degrees abduction forward simulation if appropriate

if strcmp(motion,'IntRot45')

    %Set the shoulder coordinate values
    osimModel.getCoordinateSet().get('elv_angle').setDefaultValue(deg2rad(30));
% % %     osimModel.getCoordinateSet().get('shoulder_elv').setDefaultValue(deg2rad(45));

%%%% could still set to 45 degrees but it shoudln't matter - right now this
%%%% would be affected by the custom CLF still in the model

    osimModel.getCoordinateSet().get('shoulder_elv').setDefaultValue(deg2rad(0));
    osimModel.getCoordinateSet().get('shoulder_rot').setDefaultValue(deg2rad(0));

    %Set locking on shoulder coordinates
    osimModel.getCoordinateSet().get('elv_angle').set_locked(true);
    osimModel.getCoordinateSet().get('shoulder_elv').set_locked(true);
    osimModel.getCoordinateSet().get('shoulder_rot').set_locked(false);
    
    %Finalise model connections
    osimModel.finalizeConnections();

    %Initialise the forward tool
    FwdTool = ForwardTool();

    %Settings for forward tool
    FwdTool.setModel(osimModel);
    FwdTool.setInitialTime(0); FwdTool.setFinalTime(3);

    %Set controls for simulation
    FwdTool.setControlsFileName([modelDir,'\shoulder_introt_controls.xml']);
    
    %Need to add controller set for the controls to be active in the model
    %First, clear any existing controller set from past simulations
    osimModel.getControllerSet().clearAndDestroy();
    %Add the current controller set to the model
    FwdTool.addControllerSetToModel();

    %Run forward tool
    FwdTool.run();
    
    %Calculate errors
    
    %Load in states data
    D = importdata('_states_degrees.mot');
    
    %Grab out the shoulder rotation angle
    angInd = contains(D.colheaders,'shoulder_rot') & contains(D.colheaders,'value');
    rotAng = D.data(:,angInd);
    
    %Want to penalise the model if the rotation is occurring in the wrong
    %direction, so can first check if it is going in the appropriate
    %direction for the relevant rotation. Can check this by calculating the
    %mean of data and testing whether positive or negative.
    meanRot = mean(rotAng);
    
    if meanRot > 1
        %Calculate the peak internal rotation angle
        peakRot = max(rotAng);
    else
        %Calculate the peak external rotation angle as negative
        peakRot = min(rotAng);
    end
    
    %Identify absolute error between peak and desired angle and add to total error
    isoErr = abs(desiredIntRotAng45 - peakRot);
    totalErr = totalErr + isoErr;
    
    %Cleanup
    clear D rotAng angInd peakRot isoErr
    
    %Clear forward simulation files
    delete _controls.sto _states.sto _states_degrees.mot
    
end

%% Run the external rotation at 90 degrees abduction forward simulation if appropriate

if strcmp(motion,'RotationElevation_ER')

    %Set the shoulder coordinate values
    osimModel.getCoordinateSet().get('elv_angle').setDefaultValue(deg2rad(30));
    osimModel.getCoordinateSet().get('shoulder_elv').setDefaultValue(deg2rad(90));
    osimModel.getCoordinateSet().get('shoulder_rot').setDefaultValue(deg2rad(0));

    %Set locking on shoulder coordinates
    osimModel.getCoordinateSet().get('elv_angle').set_locked(true);
    osimModel.getCoordinateSet().get('shoulder_elv').set_locked(true);
    osimModel.getCoordinateSet().get('shoulder_rot').set_locked(false);
    
    %Finalise model connections
    osimModel.finalizeConnections();

    %Initialise the forward tool
    FwdTool = ForwardTool();

    %Settings for forward tool
    FwdTool.setModel(osimModel);
    FwdTool.setInitialTime(0); FwdTool.setFinalTime(3);

    %Set controls for simulation
    FwdTool.setControlsFileName([modelDir,'\shoulder_extrot_controls.xml']);
    
    %Need to add controller set for the controls to be active in the model
    %First, clear any existing controller set from past simulations
    osimModel.getControllerSet().clearAndDestroy();
    %Add the current controller set to the model
    FwdTool.addControllerSetToModel();

    %Run forward tool
    FwdTool.run();
    
    %Calculate errors
    
    %Load in states data
    D = importdata('_states_degrees.mot');
    
    %Grab out the shoulder rotation angle
    angInd = contains(D.colheaders,'shoulder_rot') & contains(D.colheaders,'value');
    rotAng = D.data(:,angInd);
    
    %Want to penalise the model if the rotation is occurring in the wrong
    %direction, so can first check if it is going in the appropriate
    %direction for the relevant rotation. Can check this by calculating the
    %mean of data and testing whether positive or negative.
    meanRot = mean(rotAng);
    
    if meanRot < 1
        %Calculate the peak external rotation angle
        peakRot = abs(min(rotAng));
    else
        %Calculate the peak internal rotation angle as negative
        peakRot = max(rotAng)*-1;
    end
    
    %Identify absolute error between peak and desired angle and add to total error
    isoErr = abs(desiredExtRotAng90 - peakRot);
    totalErr = totalErr + isoErr;
    
    %Cleanup
    clear D rotAng angInd peakRot isoErr
    
    %Clear forward simulation files
    delete _controls.sto _states.sto _states_degrees.mot
    
end

%% Run the internal rotation at 90 degrees abduction forward simulation if appropriate

if strcmp(motion,'IntRot90')

    %Set the shoulder coordinate values
    osimModel.getCoordinateSet().get('elv_angle').setDefaultValue(deg2rad(30));
% % %     osimModel.getCoordinateSet().get('shoulder_elv').setDefaultValue(deg2rad(90));

%%%% could still set to 90 degrees but it shoudln't matter - right now this
%%%% would be affected by the custom CLF still in the model

    osimModel.getCoordinateSet().get('shoulder_elv').setDefaultValue(deg2rad(0));
    osimModel.getCoordinateSet().get('shoulder_rot').setDefaultValue(deg2rad(0));

    %Set locking on shoulder coordinates
    osimModel.getCoordinateSet().get('elv_angle').set_locked(true);
    osimModel.getCoordinateSet().get('shoulder_elv').set_locked(true);
    osimModel.getCoordinateSet().get('shoulder_rot').set_locked(false);
    
    %Finalise model connections
    osimModel.finalizeConnections();

    %Initialise the forward tool
    FwdTool = ForwardTool();

    %Settings for forward tool
    FwdTool.setModel(osimModel);
    FwdTool.setInitialTime(0); FwdTool.setFinalTime(3);

    %Set controls for simulation
    FwdTool.setControlsFileName([modelDir,'\shoulder_introt_controls.xml']);
    
    %Need to add controller set for the controls to be active in the model
    %First, clear any existing controller set from past simulations
    osimModel.getControllerSet().clearAndDestroy();
    %Add the current controller set to the model
    FwdTool.addControllerSetToModel();

    %Run forward tool
    FwdTool.run();
    
    %Calculate errors
    
    %Load in states data
    D = importdata('_states_degrees.mot');
    
    %Grab out the shoulder rotation angle
    angInd = contains(D.colheaders,'shoulder_rot') & contains(D.colheaders,'value');
    rotAng = D.data(:,angInd);
    
    %Want to penalise the model if the rotation is occurring in the wrong
    %direction, so can first check if it is going in the appropriate
    %direction for the relevant rotation. Can check this by calculating the
    %mean of data and testing whether positive or negative.
    meanRot = mean(rotAng);
    
    if meanRot > 1
        %Calculate the peak internal rotation angle
        peakRot = max(rotAng);
    else
        %Calculate the peak external rotation angle as negative
        peakRot = min(rotAng);
    end
    
    %Identify absolute error between peak and desired angle and add to total error
    isoErr = abs(desiredIntRotAng90 - peakRot);
    totalErr = totalErr + isoErr;
    
    %Cleanup
    clear D rotAng angInd peakRot isoErr
    
    %Clear forward simulation files
    delete _controls.sto _states.sto _states_degrees.mot
    
end

%%
end
%----- End of ligamentOptimiser.m -----%