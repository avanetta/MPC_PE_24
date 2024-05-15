%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2024, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% function [H, h] = lqr_maxPI(Q, R, params)
% 
%     ctrl = LQR(Q,R,params);
%     K = ctrl.K;
%     % define Autonomous system according to LQR law:
%     A_LQR = params.model.A - params.model.B*K;
% 
%     sys = LTISystem('A', A_LQR);
% 
%     % constraints are not found in params, even though they should be
%     % there???
%     % AV: I think this is due to a "access blocking". Probs the MPC-Team blocked to access in the
%     % testing function
%     % sys.x.min = [params.constraints.T1Min; params.constraints.T2Min; params.constraints.T3Min];
%     % sys.x.max = [params.constraints.T1Max; params.constraints.T2Max; params.constraints.T3Max];
%     % sys.u.min = [params.constraints.P1Min; params.constraints.P2Min];
%     % sys.u.max = [params.constraints.P1Max; params.constraints.P2Max];
% 
%     % % Get constraint via H-representation (not ideal) --> does not work
%     % with manual dimensions 6 and 4
%     % H_u = params.constraints.InputMatrix;
%     % h_u = params.constraints.InputRHS;
%     % H_x = params.constraints.StateMatrix;
%     % h_x = params.constraints.StateRHS;
%     % 
%     % T1Max = h_x(1);
%     % T1Min = -h_x(2);
%     % T2Max = h_x(3);
%     % T2Min = -h_x(4);
%     % T3Max = h_x(5);
%     % T3Min = -h_x(6);
%     % 
%     % P1Max = h_u(1);
%     % P1Min = -h_u(2);
%     % P2Max = h_u(3);
%     % P2Min = -h_u(4);
%     % 
%     % sys.x.min = [T1Min; T2Min; T3Min];
%     % sys.x.max = [T1Max; T2Max; T3Max];
%     % sys.u.min = [P1Min; P2Min];
%     % sys.u.max = [P1Max; P2Max];
% 
% 
%     % Via setConstraint (gives error because polyhedron needs to be 0 dimensional
%     % H_u = params.constraints.InputMatrix;
%     % h_u = params.constraints.InputRHS;
%     H_x = params.constraints.StateMatrix;
%     h_x = params.constraints.StateRHS;
%     % nx = params.model.nx;
%     % nu = params.model.nu;
% 
%     Px = Polyhedron(H_x, h_x);
%     % Pu = Polyhedron(H_u, h_u);
%     sys.x.with('setConstraint');
%     sys.x.setConstraint = Px;
%     % sys.u.with('setConstraint');
%     % sys.u.setConstraint = Pu;
% 
%     % Get invariant set according https://www.mpt3.org/UI/Invariance
%     InvSet = sys.invariantSet();
%     %InvSet.plot()
% 
%     % Retireve H-representation according https://www.mpt3.org/Main/HowTos
%     P = Polyhedron(InvSet);
%     %P.minHRep(); % remove redundant constraints
%     H = P.A;
%     h = P.b;
% 
% end

% %% This is Arnaud's try
% % What I realized was that constraigning x provides the correct dimensions
% % But the invariant set is the to big, because we miss the information on
% % input constraints.
% % I tried to create an augmented system and to reduce it in the end, but it
% % seems not to be correct.
% 
% function [H, h] = lqr_maxPI(Q, R, params)
% 
%     ctrl = LQR(Q,R,params);
%     K = ctrl.K;
%     %define Autonomous system according to LQR law:
%     A_LQR = params.model.A - params.model.B*K; 
% 
%     % Define augmented system
%     A_aug = [A_LQR, -params.model.B; zeros(size(params.model.B')), eye(size(params.model.B, 2))];
%     B_aug = [params.model.B; eye(size(params.model.B, 2))];
% 
%     sys = LTISystem('A', A_aug, 'B', B_aug);
% 
%     H_x = params.constraints.StateMatrix;
%     h_x = params.constraints.StateRHS;
% 
%     H_u = params.constraints.InputMatrix;
%     h_u = params.constraints.InputRHS;
% 
%     % Augment state constraints to include constraints on control inputs
%     H_aug = [H_x, zeros(size(H_x, 1), size(H_u, 2)); H_u, zeros(size(H_u, 1), size(H_x, 2))];
%     h_aug = [h_x; h_u];
% 
%     Px = Polyhedron(H_aug, h_aug);
%     sys.x.with('setConstraint');
%     sys.x.setConstraint = Px;
% 
%     % Get invariant set
%     InvSet = sys.invariantSet();
% 
%     % Project invariant set onto original state space
%     InvSet_projected = InvSet.projection(1:size(A_LQR, 2));
% 
%     % Retrieve H-representation
%     P = Polyhedron(InvSet_projected);
%     H = P.A;
%     h = P.b;
% 
% end

function [H, h] = lqr_maxPI(Q, R, params)
A = params.model.A;
B = params.model.B;

nx = params.model.nx;
nu = params.model.nu;

H_u = params.constraints.InputMatrix;
h_u = params.constraints.InputRHS;
H_x = params.constraints.StateMatrix;
h_x = params.constraints.StateRHS;

% Define LQR-System
ctrl = LQR(Q,R,params);
K = -ctrl.K;
A_LQR = A + B*K;
sys_LQR = LTISystem('A', A_LQR);

% Define constraints and convert into polyhedron (input constraints become
% new state constraints)
A_x = [H_x; K; -K];
b_x = [h_x; h_u];
Px = Polyhedron('A', A_x, 'b', b_x);

% Set same constraints into LQR-System
sys_LQR.x.with('setConstraint');
sys_LQR.x.setConstraint = Px;

% Get invariant set and H-representation
InvSet = sys_LQR.invariantSet();

H = InvSet.A;
h = InvSet.b;

end