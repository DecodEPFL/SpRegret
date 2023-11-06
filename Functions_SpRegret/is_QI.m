function [result] = is_QI(Kbin,G)
%IS_QI Summary of this function goes here
%   Detailed explanation goes here
result = true;
[ny,nu] = size(G);
G_bin = bin(G);
i = 1;
while(i<=ny && result)
    l = 1;
    while(l<=ny && result)
        j = 1;
        while(j<=nu && result)
            k = 1;
            while(k <= nu && result)
                check = Kbin(k,i)*G_bin(i,j)*Kbin(j,l)*(1-Kbin(k,l));
                if (check == 1)
                    %QI condition is not satisfied!!
                    result = false;
                end
                k = k+1;
            end
            j = j + 1;
        end
        l = l + 1;
    end
    i = i+1;

end


