%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2024, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [H, h] = lqr_maxPI(Q, R, params)
    
    ctrl = LQR(Q,R,params);
    K = ctrl.K;
    %define Autonomous system according to LQR law:
    A_LQR = params.model.A - params.model.B*K; 
    
    sys = LTISystem('A', A_LQR);

    % constraints are not found in params, even though they should be
    % there???
    % AV: I think this is due to a "access blocking". Probs the MPC-Team blocked to access in the
    % testing function
    % sys.x.min = [params.constraints.T1Min; params.constraints.T2Min; params.constraints.T3Min];
    % sys.x.max = [params.constraints.T1Max; params.constraints.T2Max; params.constraints.T3Max];
    % sys.u.min = [params.constraints.P1Min; params.constraints.P2Min];
    % sys.u.max = [params.constraints.P1Max; params.constraints.P2Max];



    % % Get constraint via H-representation (not ideal) --> does not work
    % with manual dimensions 6 and 4
    % H_u = params.constraints.InputMatrix;
    % h_u = params.constraints.InputRHS;
    % H_x = params.constraints.StateMatrix;
    % h_x = params.constraints.StateRHS;
    % 
    % T1Max = h_x(1);
    % T1Min = -h_x(2);
    % T2Max = h_x(3);
    % T2Min = -h_x(4);
    % T3Max = h_x(5);
    % T3Min = -h_x(6);
    % 
    % P1Max = h_u(1);
    % P1Min = -h_u(2);
    % P2Max = h_u(3);
    % P2Min = -h_u(4);
    % 
    % sys.x.min = [T1Min; T2Min; T3Min];
    % sys.x.max = [T1Max; T2Max; T3Max];
    % sys.u.min = [P1Min; P2Min];
    % sys.u.max = [P1Max; P2Max];


    % % Via setConstraint (gives error because polyhedron needs to be 0 dimensional
    % H_u = params.constraints.InputMatrix;
    % h_u = params.constraints.InputRHS;
    % H_x = params.constraints.StateMatrix;
    % h_x = params.constraints.StateRHS;
    % nx = params.model.nx;
    % nu = params.model.nu;
    % 
    % Px = Polyhedron(H_x, h_x);
    % Pu = Polyhedron(H_u, h_u);
    % sys.x.with('setConstraint');
    % sys.x.setConstraint = Px;
    % sys.u.with('setConstraint');
    % sys.u.setConstraint = Pu;
    
    
    % Get invariant set according https://www.mpt3.org/UI/Invariance
    InvSet = sys.invariantSet();
    %InvSet.plot()

    % Retireve H-representation according https://www.mpt3.org/Main/HowTos
    P = Polyhedron(InvSet)
    %P.minHRep(); % remove redundant constraints
    H = P.A;
    h = P.b;

end

