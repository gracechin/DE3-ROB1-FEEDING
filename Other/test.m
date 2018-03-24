load accidents
x = [255 243 0; 255 248 0].'; %Population of states
y = [0.188 -0.308 0.488; 0.188 -0.408 0.488].'; %Accidents per state
format long
X = [ones(length(x),1) x];
b1 = X\y
out = X*b1


test1 = [255.0 246.0 0.0; 255.0 246.0 0.0].';
Test1 = [ones(length(test1),1) test1];
out1 = Test1*b1