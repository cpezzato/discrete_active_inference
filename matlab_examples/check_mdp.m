function [MDP] = check_mdp(MDP)
%Adapted from https://www.fil.ion.ucl.ac.uk/spm/ by Corrado Pezzato
% Copyright (C) 2005 Wellcome Trust Centre for Neuroimaging

% Karl Friston
% $Id: spm_MDP_check.m 7766 2020-01-05 21:37:39Z karl $

% MDP structure checking
%
% MDP.V(T - 1,P,F)      - P allowable policies of T moves over F factors
% or
% MDP.U(1,P,F)          - P allowable actions at each move
% MDP.T                 - number of outcomes
%
% MDP.A{G}(O,N1,...,NF) - likelihood of O outcomes given hidden states
% MDP.B{F}(NF,NF,MF)    - transitions among hidden under MF control states
% MDP.C{G}(O,T)         - prior preferences over O outcomes in modality G
% MDP.D{F}(NF,1)        - prior probabilities over initial states
%
% MDP.a{G}              - concentration parameters for A
% MDP.b{F}              - concentration parameters for B
% MDP.d{F}              - concentration parameters for D
%
% optional:
% MDP.s(F,T)            - vector of true states - for each hidden factor
% MDP.o(G,T)            - vector of outcome     - for each outcome modality
% MDP.u(F,T - 1)        - vector of action      - for each hidden factor
% MDP.w(1,T)            - vector of precisions
%
% if C or D are not specified, they will be set to default values (of no
% preferences and uniform priors over initial steps).  If there are no
% policies, it will be assumed that I = 1 and all policies (for each
% marginal hidden state) are allowed.
%__________________________________________________________________________

% deal with a sequence of trials
%==========================================================================

% if there are multiple structures check each separately
%--------------------------------------------------------------------------
if numel(MDP) > 1
    for m = 1:size(MDP,1)
        for i = 1:size(MDP,2)
            mdp(m,i) = spm_MDP_check(MDP(m,i));
        end
    end
    MDP   = mdp;
    return
end

% fill in (posterior or  process) likelihood and priors
%--------------------------------------------------------------------------
if ~isfield(MDP,'A'), MDP.A = MDP.a; end
if ~isfield(MDP,'B'), MDP.B = MDP.b; end

% check format of likelihood and priors
%--------------------------------------------------------------------------
if ~iscell(MDP.A), MDP.A = {full(MDP.A)}; end
if ~iscell(MDP.B), MDP.B = {full(MDP.B)}; end

if isfield(MDP,'a'), if ~iscell(MDP.a), MDP.a = {full(MDP.a)}; end; end
if isfield(MDP,'b'), if ~iscell(MDP.b), MDP.b = {full(MDP.b)}; end; end


% check dimensions and orders
%==========================================================================

% numbers of transitions, policies and states
%--------------------------------------------------------------------------
Nf  = numel(MDP.B);                 % number of hidden state factors
Ng  = numel(MDP.A);                 % number of outcome factors
for f = 1:Nf
    Nu(f)    = size(MDP.B{f},3);    % number of hidden controls
    Ns(f)    = size(MDP.B{f},1);    % number of hidden states
    MDP.B{f} = double(MDP.B{f});
end
for g = 1:Ng
    No(g)    = size(MDP.A{g},1);    % number of outcomes
    MDP.A{g} = double(MDP.A{g});
end

% check policy specification (create default moving policy U, if necessary)
%--------------------------------------------------------------------------
if isfield(MDP,'U')
    if size(MDP.U,1) == 1 && size(MDP.U,3) == Nf
        MDP.U = shiftdim(MDP.U,1);
    end
end
try
    V(1,:,:) = MDP.U;               % allowable actions (1,Np)
catch
    try
        V = MDP.V;                  % allowable policies (T - 1,Np)
    catch
        
        % allowable (moving) policies using all allowable actions
        %------------------------------------------------------------------
        for f = 1:Nf
            u = 1;
            for i = 1:Nf
                if i == f
                    u = kron(1:Nu(i),u);
                else
                    u = kron(ones(1,Nu(i)),u);
                end
            end
            MDP.U(:,f)  = u;
        end
        V(1,:,:) = MDP.U;
    end
end
MDP.V = V;

% check policy specification
%--------------------------------------------------------------------------
if Nf ~= size(V,3) && size(V,3) > 1
    error('please ensure V(:,:,1:Nf) is consistent with MDP.B{1:Nf}')
end

% check preferences
%--------------------------------------------------------------------------
if ~isfield(MDP,'C')
    for g = 1:Ng
        MDP.C{g} = zeros(No(g),1);
    end
end
for g = 1:Ng
    if iscell(MDP.C)
        if isvector(MDP.C{g})
            MDP.C{g} = spm_vec(MDP.C{g});
        end
        if No(g) ~= size(MDP.C{g},1)
            error(['please ensure A{' num2str(g) '} and C{' num2str(g) '} are consistent'])
        end
    end
end

% check iinitial states
%--------------------------------------------------------------------------
if ~isfield(MDP,'D')
    for f = 1:Nf
        MDP.D{f} = ones(Ns(f),1);
    end
end
if Nf  ~= numel(MDP.D)
    error('please ensure V(:,:,1:Nf) is consistent with MDP.D{1:Nf}')
end


% check iinitial states and internal consistency
%--------------------------------------------------------------------------
if Nf  ~= numel(MDP.D)
    error('please ensure V(:,:,1:Nf) is consistent with MDP.D{1:Nf}')
end
for f = 1:Nf
    if Ns(f) ~= size(MDP.D{f},1)
        error(['please ensure B{' num2str(f) '} and D{' num2str(f) '} are consistent'])
    end
    if size(V,3) > 1
        if Nu(f) < max(spm_vec(V(:,:,f)))
            error(['please check V(:,:,' num2str(f) ') or U(:,:,' num2str(f) ')'])
        end
    end
    for g = 1:Ng
        Na  = size(MDP.A{g});
        if ~all(Na(2:end) == Ns)
            error(['please ensure A{' num2str(g) '} and D{' num2str(f) '} are consistent'])
        end
    end
end

% check probability matrices are properly specified
%--------------------------------------------------------------------------
for f = 1:Nf
    if ~all(spm_vec(any(MDP.B{f},1)))
        error(['please check B{' num2str(f) '} for missing entries'])
    end
end
for g = 1:Ng
    if ~all(spm_vec(any(MDP.A{g},1)))
        error(['please check A{' num2str(g) '} for missing entries'])
    end
end

% check initial states
%--------------------------------------------------------------------------
if isfield(MDP,'s')
    if size(MDP.s,1) > Nf
        error('please specify an initial state MDP.s for %i factors',Nf)
    end
    f  = max(MDP.s,[],2)';
    if any(f > Ns(1:numel(f)))
        error('please ensure initial states MDP.s are consistent with MDP.B')
    end
end

% check outcomes if specified
%--------------------------------------------------------------------------
if isfield(MDP,'o')
    if numel(MDP.o)
        if size(MDP.o,1) ~= Ng
            error('please specify an outcomes MDP.o for %i modalities',Ng)
        end
        if any(max(MDP.o,[],2) > No(:))
            error('please ensure outcomes MDP.o are consistent with MDP.A')
        end
    end
end

% check (primary link array if necessary)
%--------------------------------------------------------------------------
if isfield(MDP,'link')
    
    % cardinality of subordinate level
    %----------------------------------------------------------------------
    nf    = numel(MDP.MDP(1).B);               % number of hidden factors
    for f = 1:nf
        ns(f)    = size(MDP.MDP(1).B{f},1);    % number of hidden states
    end
    
    % check the size of link
    %----------------------------------------------------------------------
    if ~all(size(MDP.link) == [nf,Ng]);
        error('please check the size of link {%i,%i}',nf,Ng)
    end
    
    % convert matrix to cell array if necessary
    %----------------------------------------------------------------------
    if isnumeric(MDP.link)
        link  = cell(nf,Ng);
        for f = 1:size(MDP.link,1)
            for g = 1:size(MDP.link,2)
                if MDP.link(f,g)
                    link{f,g} = spm_speye(ns(f),No(g),0);
                end
            end
        end
        MDP.link = link;
    end
    
    % check sizes of cell array
    %----------------------------------------------------------------------
    for f = 1:size(MDP.link,1)
        for g = 1:size(MDP.link,2)
            if ~isempty(MDP.link{f,g})
                if ~all(size(MDP.link{f,g}) == [ns(f),No(g)]);
                    error('please check link{%i,%i}',f,g)
                end
            end
        end
    end
    
end

% Empirical prior preferences
%--------------------------------------------------------------------------
if isfield(MDP,'linkC')
    if isnumeric(MDP.linkC)
        linkC  = cell(numel(MDP.MDP.C),Ng);
        for f = 1:size(MDP.linkC,1)
            for g = 1:size(MDP.linkC,2)
                if MDP.linkC(f,g)
                    linkC{f,g} = spm_speye(size(MDP.MDP.C{f},1),No(g),0);
                end
            end
        end
        MDP.linkC = linkC;
    end
end

% Empirical priors over policies
%--------------------------------------------------------------------------
if isfield(MDP,'linkE')
    if isnumeric(MDP.linkE)
        linkE  = cell(1,Ng);
        for g = 1:size(MDP.linkE,2)
            if MDP.linkE(g)
                linkE{g} = spm_speye(size(MDP.MDP.E,1),No(g),0);
            end
        end
        MDP.linkE = linkE;
    end
end

% check factors and outcome modalities have proper labels
%--------------------------------------------------------------------------
for i = 1:Nf
    
    % name of factors
    %----------------------------------------------------------------------
    try
        MDP.label.factor(i);
    catch
        try
            MDP.label.factor{i} = MDP.Bname{i};
        catch
            MDP.label.factor{i} = sprintf('factor %i',i);
        end
    end
    
    % name of levels of each factor
    %----------------------------------------------------------------------
    for j = 1:Ns(i)
        try
            MDP.label.name{i}(j);
        catch
            try
                MDP.label.name{i}{j} = MDP.Sname{i}{j};
            catch
                MDP.label.name{i}{j} = sprintf('state %i(%i)',j,i);
            end
        end
    end
    
    % name of actions under each factor
    %----------------------------------------------------------------------
%     for j = 1:Nu(i)
%         try
%             MDP.label.action{i}(j);
%         catch
%             MDP.label.action{i}{j} = sprintf('act %i(%i)',j,i);
%         end
%     end
end

% name of outcomes under each modality
%--------------------------------------------------------------------------
for i = 1:Ng
    try
        MDP.label.modality(i);
    catch
        try
            MDP.label.modality{i} = MDP.Bname{i};
        catch
            MDP.label.modality{i} = sprintf('modality %i',i);
        end
    end
    for j = 1:No(i)
        try
            MDP.label.outcome{i}(j);
        catch
            try
                MDP.label.outcome{i}{j} = MDP.Oname{i}{j};
            catch
                MDP.label.outcome{i}{j} = sprintf('outcome %i(%i)',j,i);
            end
        end
    end
end

% check names are specified properly
%--------------------------------------------------------------------------
if isfield(MDP,'Aname')
    if numel(MDP.Aname) ~= Ng
        error('please specify an MDP.Aname for each modality')
    end
else
    MDP.Aname = MDP.label.modality;
end
if isfield(MDP,'Bname')
    if numel(MDP.Bname) ~= Nf
        error('please specify an MDP.Bname for each factor')
    end
else
    MDP.Bname = MDP.label.factor;
end

function [vX] = spm_vec(X,varargin)
% Vectorise a numeric, cell or structure array - a compiled routine
% FORMAT [vX] = spm_vec(X)
% X  - numeric, cell or stucture array[s]
% vX - vec(X)
%
% See spm_unvec
%__________________________________________________________________________
%
% e.g.:
% spm_vec({eye(2) 3}) = [1 0 0 1 3]'
%__________________________________________________________________________
% Copyright (C) 2005-2013 Wellcome Trust Centre for Neuroimaging

% Karl Friston
% $Id: spm_vec.m 6110 2014-07-21 09:36:13Z karl $


%error('spm_vec.c not compiled - see Makefile')

% initialise X and vX
%--------------------------------------------------------------------------
if nargin > 1
    X = [{X},varargin];
end


% vectorise numerical arrays
%--------------------------------------------------------------------------
if isnumeric(X)
    vX = X(:);

% vectorise logical arrays
%--------------------------------------------------------------------------
elseif islogical(X)
    vX = X(:);

% vectorise structure into cell arrays
%--------------------------------------------------------------------------
elseif isstruct(X)
    vX = [];
    f   = fieldnames(X);
    X    = X(:);
    for i = 1:numel(f)
        vX = cat(1,vX,spm_vec({X.(f{i})}));
    end

% vectorise cells into numerical arrays
%--------------------------------------------------------------------------
elseif iscell(X)
    vX   = [];
    for i = 1:numel(X)
        vX = cat(1,vX,spm_vec(X{i}));
    end
else
    vX = [];
end

