function err = ligamentOptimiser(x,Input)

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

    %Get model from global variable
    global osimModel

    %Get angles database from global variable
    global meanAngles angles plications

    %Extract inputs
    plication = Input.plicationType;
    motion = Input.motionType;
    modelDir = Input.modelDir;

    %Set the ligament parameters depending on the motion

    %Check if its an internal rotation motion
    if contains(motion,'IntRot')

        %The variables input to the optimisation are:
        %[1] Shoulder rotation ligament upper limit
        %[2] Shoulder rotation ligament upper stiffness
        %[3] Shoulder rotation ligament transition
        CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).set_upper_limit(x(1));
        CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).set_upper_stiffness(x(2));
        CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).set_transition(x(3));
        
        %There is the potential that the upper limit for internal shoulder
        %rotation may be *lower* than the lower limit, so we can just set
        %this to a low value so it is ignored in the internal rotation sims
        CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).set_lower_limit(-90);

    %Check if its an external rotation motion
    elseif contains(motion,'ExtRot')

        %The variables input to the optimisation are:
        %[1] Shoulder rotation ligament lower limit
        %[2] Shoulder rotation ligament lower stiffness
        %[3] Shoulder rotation ligament transition
        CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).set_lower_limit(x(1));
        CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).set_lower_stiffness(x(2));
        CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).set_transition(x(3));
        
        %There is the potential that the lower limit for internal shoulder
        %rotation may be *higher* than the upper limit, so we can just set
        %this to a high value so it is ignored in the external rotation sims
        CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_rot_ligaments')).set_upper_limit(90);

    %Its an elevation motion
    else

        %The variables input to the optimisation are:
        %[1] Shoulder elevation ligament upper limit
        %[2] Shoulder elevation ligament upper stiffness
        %[3] Shoulder elevation ligament transition
        CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_elv_ligaments')).set_upper_limit(x(1));
        CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_elv_ligaments')).set_upper_stiffness(x(2));
        CoordinateLimitForce.safeDownCast(osimModel.getForceSet.get('shoulder_elv_ligaments')).set_transition(x(3));

    end

    %Set the desired joint angle for the motion and plication type
    %Get the row (plication) and column (motion) indices for angle value
    rowLog = logical(strcmp(plication,plications));
    colLog = logical(strcmp(motion,angles));
    desiredAng = meanAngles(rowLog,colLog);

    %Set the desired angles and joint locking for the current simulation

    %Check if its an internal rotation motion
    if contains(motion,'Rot')

        %Check for level of shoulder elevation.
        %NOTE: given that at this point the shoulder elevation won't change the
        %rotational ligaments force capacity, the setting of shoulder elevation
        %isn't necessary - but it's nice to do anyway
        if contains(motion,'Rot0')
            %Set the shoulder coordinate values
            osimModel.getCoordinateSet().get('elv_angle').setDefaultValue(deg2rad(30));
            osimModel.getCoordinateSet().get('shoulder_elv').setDefaultValue(deg2rad(0));
            osimModel.getCoordinateSet().get('shoulder_rot').setDefaultValue(deg2rad(0));
        elseif contains(motion,'Rot45')
            %Set the shoulder coordinate values
            osimModel.getCoordinateSet().get('elv_angle').setDefaultValue(deg2rad(30));
            osimModel.getCoordinateSet().get('shoulder_elv').setDefaultValue(deg2rad(45));
            osimModel.getCoordinateSet().get('shoulder_rot').setDefaultValue(deg2rad(0));        
        elseif contains(motion,'Rot90')
            %Set the shoulder coordinate values
            osimModel.getCoordinateSet().get('elv_angle').setDefaultValue(deg2rad(30));
            osimModel.getCoordinateSet().get('shoulder_elv').setDefaultValue(deg2rad(90));
            osimModel.getCoordinateSet().get('shoulder_rot').setDefaultValue(deg2rad(0)); 
        end

        %Lock all coordinates except for shoulder rotation
        osimModel.getCoordinateSet().get('elv_angle').set_locked(true);
        osimModel.getCoordinateSet().get('shoulder_elv').set_locked(true);
        osimModel.getCoordinateSet().get('shoulder_rot').set_locked(false);

    %Its an elevation motion
    else

        %Check for elevation angle (i.e. abduction/flexion)
        %NOTE: given that at this point the elevation angle won't change the
        %elevation ligaments force capacity, the setting of elevation angle
        %isn't necessary - but it's nice to do anyway
        if strcmp(motion,'Abduction')
            %Set the shoulder coordinate values
            osimModel.getCoordinateSet().get('elv_angle').setDefaultValue(deg2rad(30));
            osimModel.getCoordinateSet().get('shoulder_elv').setDefaultValue(deg2rad(0));
            osimModel.getCoordinateSet().get('shoulder_rot').setDefaultValue(deg2rad(0));
        elseif strcmp(motion,'Flexion')
            %Set the shoulder coordinate values
            osimModel.getCoordinateSet().get('elv_angle').setDefaultValue(deg2rad(90));
            osimModel.getCoordinateSet().get('shoulder_elv').setDefaultValue(deg2rad(0));
            osimModel.getCoordinateSet().get('shoulder_rot').setDefaultValue(deg2rad(0));        
        end

        %Lock all coordinates except for shoulder elevation
        osimModel.getCoordinateSet().get('elv_angle').set_locked(true);
        osimModel.getCoordinateSet().get('shoulder_elv').set_locked(false);
        osimModel.getCoordinateSet().get('shoulder_rot').set_locked(true);

    end

    %Finalise model connections
    osimModel.finalizeConnections();

    %% Run the forward simulation

    %Initialise the forward tool
    FwdTool = ForwardTool();

    %Settings for forward tool
    FwdTool.setModel(osimModel);

    %Set times based on motion. Rotations = 3 seconds; Elevations = 5 seconds
    if contains(motion,'Rot')
        FwdTool.setInitialTime(0); FwdTool.setFinalTime(3);
    else
        FwdTool.setInitialTime(0); FwdTool.setFinalTime(5);
    end

    %Set controls for simulation, depending on whether it is an elevation or
    %internal/external rotation motion
    if contains(motion,'IntRot')
        FwdTool.setControlsFileName([modelDir,'\shoulder_introt_controls.xml']);
    elseif contains(motion,'ExtRot')
        FwdTool.setControlsFileName([modelDir,'\shoulder_extrot_controls.xml']);
    else
        FwdTool.setControlsFileName([modelDir,'\shoulder_elv_controls.xml']);
    end

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

    %Grab out the relevant shoulder angle
    if contains(motion,'Rot')
        angInd = contains(D.colheaders,'shoulder_rot') & contains(D.colheaders,'value');
    else
        angInd = contains(D.colheaders,'shoulder_elv') & contains(D.colheaders,'value');
    end
    jointAng = D.data(:,angInd);

    %Calculate the peak angle achieved
    if contains(motion,'ExtRot')
        peakAng = min(jointAng);
    else
        peakAng = max(jointAng);
    end

    %Identify the error between the achieved and desired angle
    err = abs(desiredAng - peakAng);

    %Clear forward simulation files
    delete _controls.sto _states.sto _states_degrees.mot

end
%----- End of ligamentOptimiser.m -----%