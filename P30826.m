% Problem 3

firstnumber = input('Enter the first integer: ');
secondnumber = input('Enter the second integer: ');

result = oddEvenOperation(firstnumber, secondnumber);



function result = oddEvenOperation(firstnumber, secondnumber)
    if mod(firstnumber, 2) ~= 0 && mod(secondnumber, 2) ~= 0
        result = firstnumber + secondnumber;
    elseif mod(firstnumber, 2) == 0 && mod(secondnumber, 2) == 0
        result = abs(firstnumber - secondnumber);
    else
        result = firstnumber * secondnumber;
    end
    disp(['Result = ', num2str(result)])
end