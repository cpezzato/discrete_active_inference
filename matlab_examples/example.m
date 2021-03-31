%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  Active inference for task planning AIRLab Demo
%
%  Author:  Corrado Pezzato
%  Date:    17.07.2020
%
%  Optimized ersion to be ported in Python
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc
clear
close all

%% Initialization of the model structure for state estimation only ========

% Initialization can be done at the beginning when defining the objects

% Initial Observations: o
%--------------------------------------------------------------------------
%% THIS SHOULD BE PART OF ANOTHER NODE WHICH BROEADC
% Observations (these can be initialised with getObservations in ROS). The
% current situation means the robot is not nearby the object, is not
% holding anything, and is not at the home location
o_h1 = 2; % isHolding is False
o_r1 = 2; % isReachable is False
o_l1 = 2; % isAt is False

% Initialise mdp models for state estimation
%--------------------------------------------------------------------------
% These come from the BT designed offline, they keep track of the staes and
% allow for action selection
mdp_h1 = init_isHolding(o_h1);
mdp_r1 = init_isReachable(o_r1);
mdp_l1 = init_isAt(o_l1);

%% Initialization of the model structure for action selection =============

spm_norm([1.5, 0.5]')

MDP_h1 = mdp_h1;
MDP_r1 = mdp_r1;
MDP_l1 = mdp_l1;

% Remove actions but for idle for state estimation
% mdp_h1.V = 1;
% mdp_r1.V = 1;
% mdp_l1.V = 1;

% Remove observations from MDP structures 
MDP_h1 = rmfield(MDP_h1,'o');
MDP_r1 = rmfield(MDP_r1,'o');
MDP_l1 = rmfield(MDP_l1,'o');
MDP_h1 = rmfield(MDP_h1,'d');
MDP_r1 = rmfield(MDP_r1,'d');
MDP_l1 = rmfield(MDP_l1,'d');
    
% Other initializations for simulation purposes only. To be remove in ROS
%--------------------------------------------------------------------------
epochs = 45; % How many times the simulation goes on
N = 3; % Steps to get an action's effect 
% Auxiliary variables to keep track of actions' success and effects
prevAction = 3; % 3 means null
% Initialise counter to keep track of persistent actions
counter = 0;

%% Behavior execution

% Add here the prior to achieve a goal
%--------------------------------------------------------------------------
MDP_h1.C{1} = [1 0]';
MDP_r1.C{1} = [0 0]';
MDP_l1.C{1} = [0 0]';
%--------------------------------------------------------------------------
% Set variables for figures and plots
Pix_SS = get(0,'screensize');
set(gcf,'position',[Pix_SS(3)-640 Pix_SS(3)-480 640 480]);

state_index = ['s_h1','s_r1','s_l1'];
% Action index   s_h1   s_r1  s_l1
action_index = [1 2 3; 1 4 1; 1 5 1]; 
% 1: Idle
% 2: Pick
% 3: Place
% 4: Move

%% Loop for active inference using MDPs ===================================
% All mdps and MDPs. This lower case is for perception only
    allmdps{1} = mdp_h1;
    allmdps{2} = mdp_r1;
    allmdps{3} = mdp_l1;
    
    allMDPs{1} = MDP_h1;
    allMDPs{2} = MDP_r1;
    allMDPs{3} = MDP_l1; 
    
% Loop for several iterations
for i=1:epochs
    
    % This is what should happen every time we get a tick from the BT 
    %% State estimation
    
    % Get new observations
    %----------------------------------------------------------------------
    for p = 1:numel(allMDPs)
        allmdps{p} = aip(allmdps{p});
    end
    
    % Set current belief state D as the belief d for action selection
    %----------------------------------------------------------------------    
    for p = 1:numel(allMDPs)
        allMDPs{p}.D{1} = allmdps{p}.d{1};
    end
    
    % At each new iteration you restore all available actions.
    allMDPs{1}.E = spm_norm([1.01 1 1]'); % Idle, pick, place
    allMDPs{2}.E = spm_norm([1.01 1]');   % Idle, move MPC
    allMDPs{3}.E = spm_norm([1.01 1]');   % Idle, move Base
    
    % Check if previously pushed priors have been satisfied 
    % If so, remove their preconditions from global prior.
    
    % For each prior > 1, if the current desired state is met set that value to 0
    %----------------------------------------------------------------------
    for p = 1:numel(allMDPs)
        for k = 1:2
            if (allMDPs{p}.C{1}(k) == 2) && (k == allmdps{p}.s)
                % Remove precondition pushed since met
                allMDPs{p}.C{1}(k) = 0; 
            end
        end
    end
    
    %% HERE I HAVE TO LOOP OVER THE MDPs which has active priors
    % Also, I should define preconditions in the form of priors such that I
    % can just sum them
    
    % In Python for all MDPs select only the ones with active priors
    % Thereshould be one at the beginning of the loop, the next should
    % appear only if priors are pushed automatically. 
    
    % Initialize variable to contain the selectd actions from the active mdps
    temp_u = []; 
    
    % Find active MDPs: for _mdp in allMDP: if min(_mdp.C)>0 then
    % activeMDP = activeMDP + MDP;
    for p = 1:numel(allMDPs)
        if max(max(allMDPs{p}.C{1})) > 0
            allMDPs{p} = aip(allMDPs{p});   
            temp_u = [temp_u allMDPs{p}.u];
        end
    end
    
    % If none of the above ran since no prior is given
    if isempty(temp_u)
        temp_u = 1;
    end 
    % } 
    
    % Check if both actions are null, if so it means we achieved the
    % objective and no further actions are needed
    % This check if there is any action which is not idle
    % If both actions are null it means we achieved the goal
    if max(temp_u) == 1 && min(temp_u) == 1
        fprintf('We can stop here, return SUCCESS \n\n')
        % Plot last state of the system
        for p = 1:numel(allMDPs)
            allMDPs{p}.o = allmdps{p}.o;
        end
        mdpPlot(allMDPs{1},allMDPs{2},allMDPs{3},1);
        break % This break can be removed in production code
    else 
        
    %% Reactivity to unforeseen events ====================================
    %----------------------------------------------------------------------
    % Flag to check if an action has been found    
      actionFound = 0;
    % For every of the possible selected actions which are not idle, I need to check which one
    % I can execute, if not I need to push its preconditions to the
    % priors
      while ~actionFound  
        % for _mdp in nonIdleMDP: 
        % check preconditions action(u)
        % if met, set action found (break) and execute
        % else, push preconditions of the action and remove from available
        % ones
        for p = 1:numel(allMDPs)
            if isfield(allMDPs{p},'u') && max(max(allMDPs{p}.C{1})) > 0 %% AND IS ACTIVE
                if allMDPs{p}.u ~= 1
                    curr_u = allMDPs{p}.u;
                    prec = allMDPs{p}.Prec{curr_u};
                    unmet_prec = 0;
                    % Check precondtions, for each row of .Prec
                    for j = 1:size(prec,1) % If there are no preconditions this is skipped 
                        % To add also a check if the precondition is not 0,
                        % python doesn not allow empty stuff
                        if allmdps{prec(j,1)}.s ~= prec(j,2) % Select type of MDP of interest and check the state
                          unmet_prec = 1; % If not the same, we have an unmet precondition
                          allMDPs{prec(j,1)}.C{1}(prec(j,2)) = 2; % Set the prior for that precondition
                          % Inhibit this action for the inner loop
                          allMDPs{p}.E(curr_u) = 0;
                        end
                    end
                    if ~unmet_prec % I need to prevent to set idle action as action found
                        currAction = action_index(p,curr_u);
                        actionFound = 1;
                        break
                    end
                end  
            end
        end

    % Exploration with new local prior preferences
    %------------------------------------------------------------------
    % Run active inference with updated prior and allowable actions
    % Find active MDPs: for _mdp in allMDP: if min(_mdp.C)>0 then
    % activeMDP = activeMDP + MDP;
    temp_u = [];
    for p = 1:numel(allMDPs)
        if max(max(allMDPs{p}.C{1})) > 0
            allMDPs{p} = aip(allMDPs{p});   
            temp_u = [temp_u allMDPs{p}.u];
        end
    end
    
    % If none of the above ran since no prior is given
    if isempty(temp_u)
        temp_u = 1;
    end 
    % }
        % If both actions are still null action, return failure. This means
        % that there are no suitable actions to meet the necessary
        % preconditions
        if max(temp_u) == 1 && min(temp_u) == 1
            fprintf('No suitable action found\n\n')
            fprintf('Return FAILURE\n\n')
            actionFound = 1;
            break
        else
        end
      end
    end
    %% Extra operations ===================================================
    %----------------------------------------------------------------------
    
    % Change observations according to the effects of the selected actions. 
    % We assume that an action has to persist for 3 time steps before its 
    % effects are visible in the environemnt. 
    % This simulates non-instantaneous actions then
    %----------------------------------------------------------------------
    if prevAction == currAction % Persistence
        counter = counter+1;
    else
        % When an action changes then reset counter for persistence
        counter = 0;
    end
    
    % Check if an action is persisting more than N steps, then its effects
    % change the observations.     
    if counter > N
        switch currAction
            case 1 % Idle
                % Do nothing
            case 2 % Pick
                allmdps{1}.o = 1;   % Holding something              
            case 3 % Place
                allmdps{1}.o = 2;   % Free hand 
            case 4 % Move MPC
                allmdps{2}.o = 1;   % Reachable location   
            case 5 % Move base
                allmdps{3}.o = 1;   % Base at location        
        end
    end
        
    % Update prevAction for next iteration
    prevAction = currAction;
    
    % Visualize data
    %----------------------------------------------------------------------
    % Plot
    for p = 1:numel(allMDPs)
        allMDPs{p}.o = allmdps{p}.o;
    end
    mdpPlot(allMDPs{1},allMDPs{2},allMDPs{3},currAction);
    for p = 1:numel(allMDPs)
        allMDPs{p} = rmfield(allMDPs{p},'o');
    end
    
end

%% Auxiliary Functions ====================================================
function A = spm_norm(A)
    % Normalisation of probability matrix (column elements sum to 1)
    %----------------------------------------------------------------------
    % The function goes column by column and it normalise such that 
    % elements of each colums sum to 1

    for i = 1:size(A,2)     
        for j = 1:size(A,3)
            for k = 1:size(A,4)
                for l = 1:size(A,5)
                    S = sum(A(:,i,j,k,l),1);
                    if S > 0
                        A(:,i,j,k,l) = A(:,i,j,k,l)/S;
                    else
                        A(:,i,j,k,l) = 1/size(A,1);
                    end
                end
            end
        end
    end
end

function mdp = init_isHolding(o)
% Returns the mdp structure for a state of type isHolding(). Takes as input
% the initial oservation o, and the parameters par

    % Initialization of the mdp for state isHolding
    label.factor     = {'holdingSomething'}; % These 3 might be removed once got rid of the mdp check
    label.modality   = {'what'};
    label.outcome{1} = {'yep','no'};
    label.actions = {'idle','pick','place'};

    % Initial prior: (negative cost) C, initialised to zero
    %----------------------------------------------------------------------
    C = spm_norm([0 0]'); % True, False

    % Initial guess about the states: d, all equally possible
    %----------------------------------------------------------------------
    d{1} = [0.5 0.5]';
    
    % Probabilistic mapping from hidden states to outcomes: A
    %----------------------------------------------------------------------
    A{1}(:,:,1) = eye(2);
    
    % Transition matrix: B
    %----------------------------------------------------------------------
    % This matrix indicates the available actions that the agent has and 
    % how these change (ideally) the states. 
    B{1}(:,:,1) = eye(2);       % Null action, keeps same state
    B{1}(:,:,2) = [1 1; 0 0];   % pick(obj, objLoc): a_pk makes isHolding 
                                % true 
    B{1}(:,:,3) = [0 0; 1 1];   % place(obj): a_pl makes isHolding 
                                % false
                                
    % Allowable policies: V and Prior over policies: e
    %----------------------------------------------------------------------
    % V indicates the indexes of the policies that are available to the
    % agent. This is just for convenience in the code, could be derived
    % from B
    V(:,:,1) = [1 2 3]; % Three actions

    % Prior over policy is used to give more probability (chance of 
    % success) to some of the control actions. Null action is somehow
    % preferred slightly
    e = spm_norm([1.01 1 1])';
    
    % Define MDP structure
    %----------------------------------------------------------------------
    mdp.A = A;              % Observation model or likelihood
    mdp.B = B;              % Transition probabilities
    mdp.C{1} = C;           % Prior over outcomes (preferred goal)
    mdp.d = d;              % Prior over initial states
    mdp.V = V;              % Allowable policies
    mdp.o = o;              % Observation
    mdp.label = label;      % Names and labels
    mdp.E = e;              % Prior over policies (preferred ones)
    mdp.kappa_d = .2;         % Learning rate for initial states

    % Preconditions of actions
    mdp.Prec{1}= []; % There are no preconditions for idle
    mdp.Prec{2}= [2 1; 1 2]; % Action pick: [isReachable; handFree]
    mdp.Prec{3}= []; % Action place, nothing so far.
    
    % Note: kappa_d is an additional learning rate for
    % we are basically continuously updating the belief about d with stream
    % of data as input. An important thing is that we need to normalise the
    % value at every iteration otherwise we are incrementing the eveidence 
    % unboundtly and whenever the observation changes it takes quite a lot 
    % of time to converge to the correct value. To reach 50% it would 
    % require the same amount of opposite observations. We use then 
    % normalised values and we adopt a learning rate for the belief 
    % update MDP.kappa_d

    % Check correctness of mdp structure
    mdp = check_mdp(mdp);
end

function mdp = init_isAt(o)
% Returns the mdp structure for a state of type isAt(). Takes as input
% the initial oservation o, and the parameters par

    % Initialization of the mdp for state isHolding
    label.factor     = {'isSomewhere'};
    label.modality   = {'what'};
    label.outcome{1} = {'yep','no'};
    label.actions = {'idle','move'};

    % Initial prior: (negative cost) C, initialised to zero
    %----------------------------------------------------------------------
    C = spm_norm([0 0]'); % True, False

    % Initial guess about the states: d, all equally possible
    %----------------------------------------------------------------------
    d{1} = [0.5 0.5]';
    
    % Probabilistic mapping from hidden states to outcomes: A
    %----------------------------------------------------------------------
    A{1}(:,:,1) = eye(2);
    
    % Transition matrix: B
    %----------------------------------------------------------------------
    B{1}(:,:,1) = eye(2);       % Null action, keeps same state
    B{1}(:,:,2) = [1 1; 0 0];   % move(loc): a_mv makes isAt true 

    % Allowable policies: V and Prior over policies: e
    %----------------------------------------------------------------------
    % V indicates the indexes of the policies that are available to the
    % agent. This is just for convenience in the code, could be derived
    % from B
    V(:,:,1) = [1 2]; % Two actions

    % Prior over policy is used to give more probability (chance of 
    % success) to some of the control actions. Null action is somehow
    % preferred slightly
    e = spm_norm([1.01 1]');
    
    % Define MDP structure
    %----------------------------------------------------------------------
    mdp.A = A;              % Observation model or likelihood
    mdp.B = B;              % Transition probabilities
    mdp.C{1} = C;           % Prior over outcomes (preferred goal)
    mdp.d = d;              % Prior over initial states
    mdp.V = V;              % Allowable policies
    mdp.o = o;              % Observation
    mdp.label = label;      % Names and labels
    mdp.E = e;              % Prior over policies (preferred ones)
    mdp.kappa_d = .2;         % Learning rate for initial states
    % Check correctness of mdp structure
    mdp = check_mdp(mdp);
    
    % Preconditions of actions
    mdp.Prec{1}= []; % There are no preconditions for idle
    mdp.Prec{2}= []; % Action move: none for now, might add pathFree later
end

function mdp = init_isReachable(o)
% Returns the mdp structure for a state of type isHolding(). Takes as input
% the initial oservation o, and the parameters par

    % Initialization of the mdp for state isHolding
    label.factor     = {'isReachable'};
    label.modality   = {'what'};
    label.outcome{1} = {'yep','no'};
    label.actions = {'idle','move'};
    
    % Initial prior: (negative cost) C, initialised to zero
    %----------------------------------------------------------------------
    C = spm_norm([0 0]'); % True, False

    % Initial guess about the states: d, all equally possible
    %----------------------------------------------------------------------
    d{1} = [0.5 0.5]';
    
    % Probabilistic mapping from hidden states to outcomes: A
    %----------------------------------------------------------------------
    A{1}(:,:,1) = eye(2);
    
    % Transition matrix: B
    %----------------------------------------------------------------------
    B{1}(:,:,1) = eye(2);       % Idle action, keeps same state
    B{1}(:,:,2) = [1 1; 0 0];   % move(loc): a_mv makes isAt true 

    % Allowable policies: V and Prior over policies: e
    %----------------------------------------------------------------------
    % V indicates the indexes of the policies that are available to the
    % agent. This is just for convenience in the code, could be derived
    % from B
    V(:,:,1) = [1 2]; % Two actions

    % Prior over policy is used to give more probability (chance of 
    % success) to some of the control actions. Idle action is somehow
    % preferred slightly
    e = spm_norm([1.01 1]');
    
    % Define MDP structure
    %----------------------------------------------------------------------
    mdp.A = A;              % Observation model or likelihood
    mdp.B = B;              % Transition probabilities
    mdp.C{1} = C;           % Prior over outcomes (preferred goal)
    mdp.d = d;              % Prior over initial states
    mdp.V = V;              % Allowable policies
    mdp.o = o;              % Observation
    mdp.label = label;      % Names and labels
    mdp.E = e;              % Prior over policies (preferred ones)
    mdp.kappa_d = .2;         % Learning rate for initial states
    % Check correctness of mdp structure
    mdp = check_mdp(mdp);
    
    % Preconditions of actions
    mdp.Prec{1}= []; % There are no preconditions for idle
    mdp.Prec{2}= []; % Action move: none for now, might add pathFree later

end

function mdpPlot(mdp_h,mdp_r,mdp_l,currAction)
    % Animated plot for state estimation, current outcome and selected
    % action
    %----------------------------------------------------------------------
    % Build observations in a vector form
    o_h_plot = [0 0]';
    o_r_plot = [0 0]';
    o_l_plot = [0 0]';

    o_h_plot(mdp_h.o(1)) = 1;
    o_r_plot(mdp_r.o(1)) = 1;
    o_l_plot(mdp_l.o(1)) = 1;

    % Build vector containing the selected action for the next time step
    agent_u = zeros(5,1);
    agent_u(currAction) = 1;

    % Plots
    subplot(2,2,1)
    xbar_label = categorical({'isHolding','!isHolding',...
                              'isReachable','!isReachable'...
                              'isAt','!isAt'});
                          
    bar(xbar_label,[spm_norm(mdp_h.D{1});spm_norm(mdp_r.D{1});spm_norm(mdp_l.D{1})])
    ylim([0 1])
    ylabel('Probability of a state')
    xlabel('States')
    title('State estimation')
    drawnow
    
    subplot(2,2,2)
    xbar_label = categorical({'isHolding','!isHolding',...
                              'isReachable','!isReachable'...
                              'isAt','!isAt'});
    bar(xbar_label,[o_h_plot;o_r_plot;o_l_plot])
    ylim([0 1.4])
    ylabel('[-]')
    xlabel('Outcomes')
    title('Sensed outcome')
    drawnow
    
    subplot(2,2,3)
    xbar_label = categorical({'Idle','Pick','Place','Move MPC','MoveBase'});        
    bar(xbar_label,agent_u)
    ylim([0 1.4])
    ylabel('[-]')
    xlabel('Actions')
    title('Selected action')
    drawnow
    
    subplot(2,2,4)
    xbar_label = categorical({'isHolding','!isHolding',...
                              'isReachable','!isReachable'...
                              'isAt','!isAt'});
    bar(xbar_label,[mdp_h.C{1};mdp_r.C{1};mdp_l.C{1}])
    ylim([0 5])
    ylabel('[-]')
    xlabel('Prior over outcomes')
    title('Preferences')
    drawnow
end