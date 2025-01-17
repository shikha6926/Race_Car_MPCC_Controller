%
% Copyright 2019 Gianluca Frison, Dimitris Kouzoupis, Robin Verschueren,
% Andrea Zanelli, Niels van Duijkeren, Jonathan Frey, Tommaso Sartor,
% Branimir Novoselnik, Rien Quirynen, Rezart Qelibari, Dang Doan,
% Jonas Koenemann, Yutao Chen, Tobias Schöls, Jonas Schlagenhauf, Moritz Diehl
%
% This file is part of acados.
%
% The 2-Clause BSD License
%
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are met:
%
% 1. Redistributions of source code must retain the above copyright notice,
% this list of conditions and the following disclaimer.
%
% 2. Redistributions in binary form must reproduce the above copyright notice,
% this list of conditions and the following disclaimer in the documentation
% and/or other materials provided with the distribution.
%
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
% AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
% IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
% ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
% LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
% CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
% SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
% INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
% CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
% ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
% POSSIBILITY OF SUCH DAMAGE.;
%

SOURCES = { ...
            'pacejka_model_model/pacejka_model_expl_ode_fun.c', ...
            'pacejka_model_model/pacejka_model_expl_vde_forw.c',...
            'pacejka_model_cost/pacejka_model_cost_ext_cost_0_fun.c',...
            'pacejka_model_cost/pacejka_model_cost_ext_cost_0_fun_jac.c',...
            'pacejka_model_cost/pacejka_model_cost_ext_cost_0_fun_jac_hess.c',...
            'pacejka_model_cost/pacejka_model_cost_ext_cost_fun.c',...
            'pacejka_model_cost/pacejka_model_cost_ext_cost_fun_jac.c',...
            'pacejka_model_cost/pacejka_model_cost_ext_cost_fun_jac_hess.c',...
            'pacejka_model_cost/pacejka_model_cost_ext_cost_e_fun.c',...
            'pacejka_model_cost/pacejka_model_cost_ext_cost_e_fun_jac.c',...
            'pacejka_model_cost/pacejka_model_cost_ext_cost_e_fun_jac_hess.c',...
            'pacejka_model_constraints/pacejka_model_constr_h_fun.c', ...
            'pacejka_model_constraints/pacejka_model_constr_h_fun_jac_uxt_zt_hess.c', ...
            'pacejka_model_constraints/pacejka_model_constr_h_fun_jac_uxt_zt.c', ...
            'acados_solver_sfunction_pacejka_model.c', ...
            'acados_solver_pacejka_model.c'
          };

INC_PATH = '/acados/include';

INCS = {['-I', fullfile(INC_PATH, 'blasfeo', 'include')], ...
        ['-I', fullfile(INC_PATH, 'hpipm', 'include')], ...
        ['-I', fullfile(INC_PATH, 'acados')], ...
        ['-I', fullfile(INC_PATH)]};



CFLAGS = 'CFLAGS=$CFLAGS';
LDFLAGS = 'LDFLAGS=$LDFLAGS';
COMPFLAGS = 'COMPFLAGS=$COMPFLAGS';
COMPDEFINES = 'COMPDEFINES=$COMPDEFINES';



LIB_PATH = ['-L', fullfile('/acados/lib')];

LIBS = {'-lacados', '-lhpipm', '-lblasfeo'};

% acados linking libraries and flags
    

mex('-v', '-O', CFLAGS, LDFLAGS, COMPFLAGS, COMPDEFINES, INCS{:}, ...
    LIB_PATH, LIBS{:}, SOURCES{:}, ...
    '-output', 'acados_solver_sfunction_pacejka_model' );

fprintf( [ '\n\nSuccessfully created sfunction:\nacados_solver_sfunction_pacejka_model', '.', ...
    eval('mexext')] );


%% print note on usage of s-function
fprintf('\n\nNote: Usage of Sfunction is as follows:\n')
input_note = 'Inputs are:\n';
i_in = 1;
input_note = strcat(input_note, num2str(i_in), ') lbx_0 - lower bound on x for stage 0,',...
                    ' size [9]\n ');
i_in = i_in + 1;
input_note = strcat(input_note, num2str(i_in), ') ubx_0 - upper bound on x for stage 0,',...
                    ' size [9]\n ');
i_in = i_in + 1;
input_note = strcat(input_note, num2str(i_in), ') parameters - concatenated for all shooting nodes 0 to N+1,',...
                    ' size [820]\n ');
i_in = i_in + 1;
input_note = strcat(input_note, num2str(i_in), ') lbx for shooting nodes 1 to N-1, size [351]\n ');
i_in = i_in + 1;
input_note = strcat(input_note, num2str(i_in), ') ubx for shooting nodes 1 to N-1, size [351]\n ');
i_in = i_in + 1;
input_note = strcat(input_note, num2str(i_in), ') lbu for shooting nodes 0 to N-1, size [120]\n ');
i_in = i_in + 1;
input_note = strcat(input_note, num2str(i_in), ') ubu for shooting nodes 0 to N-1, size [120]\n ');
i_in = i_in + 1;
input_note = strcat(input_note, num2str(i_in), ') lh, size [1]\n ');
i_in = i_in + 1;
input_note = strcat(input_note, num2str(i_in), ') uh, size [1]\n ');
i_in = i_in + 1;

fprintf(input_note)

disp(' ')

output_note = 'Outputs are:\n';
i_out = 0;
i_out = i_out + 1;
output_note = strcat(output_note, num2str(i_out), ') u0, control input at node 0, size [3]\n ');
i_out = i_out + 1;
output_note = strcat(output_note, num2str(i_out), ') acados solver status (0 = SUCCESS)\n ');
i_out = i_out + 1;
output_note = strcat(output_note, num2str(i_out), ') KKT residual\n ');
i_out = i_out + 1;
output_note = strcat(output_note, num2str(i_out), ') x1, state at node 1\n ');
i_out = i_out + 1;
output_note = strcat(output_note, num2str(i_out), ') CPU time\n ');
i_out = i_out + 1;
output_note = strcat(output_note, num2str(i_out), ') SQP iterations\n ');

fprintf(output_note)
