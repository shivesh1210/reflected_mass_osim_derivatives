function [X] = se3ToR6(A)
[X] = [-A(2, 3), A(1, 3), -A(1, 2), A(1, 4), A(2, 4), A(3, 4)];
end

