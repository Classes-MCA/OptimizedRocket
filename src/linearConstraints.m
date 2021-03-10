clear;

x = ones(1,10);

for i = 1:length(x)
    
    A(i,i) = -2;
    A(i,i+1) = 2;
    A(i,i+2) = -1;
    A(i,i+3) = 1;
    
end

disp(A)