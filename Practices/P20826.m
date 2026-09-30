% Problem 2
% Equations:
% 2x + y = 5
% x - y = 1

A = [2, 1; 1, -1];
B = [5; 1];

solution = A \ B;

x = solution(1);
y = solution(2);

disp('Equations:');
disp('2x + y = 5');
disp('x - y = 1');
disp('Solution:');
disp(['x = ', num2str(x)])
disp(['y = ', num2str(y)])