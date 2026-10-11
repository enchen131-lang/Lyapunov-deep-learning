function output = actFun(x, r, caseFun)

switch (caseFun)
    case 1
        %% AF I
        output = x;
    case 2
        %% AF II
        output = (abs(x).^r).*sign(x);
    case 3
        %% AF III
        output = x + (abs(x).^r).*sign(x);
    case 4
        %% AF IV
        output = x + (abs(x).^r + abs(x).^(1/r)).*sign(x);
    otherwise
        output = x;
end
end