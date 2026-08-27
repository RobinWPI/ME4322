% Problem 4

numberOfValues = 10;
xValues = 1:numberOfValues;
yValues = zeros(1, numberOfValues);

for index = 1:numberOfValues
    yValues(index) = rand;
end

figure;
plot(xValues, yValues, 'o', 'LineWidth', 1, 'MarkerSize', 10);
grid on;
xlabel('Random Number Index');
ylabel('Random Number');
title('Ten Random Numbers');
xticks(xValues);