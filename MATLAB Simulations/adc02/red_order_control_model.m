% Open the Simulink model
open_system('for_ssmodel');

% Perform linearization
[A, B, C, D] = linearize('for_ssmodel');

% Display the state-space representation
disp('State-space matrices:');
disp('A =');
disp(A);
disp('B =');
disp(B);
disp('C =');
disp(C);
disp('D =');
disp(D);
