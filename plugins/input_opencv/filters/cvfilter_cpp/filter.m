clc
clear

data = csvread('1.csv')
[theat_s,t_index] = sort(data(:,1))
%[rho,r_index] = sort(a(:,1))

theat = data(:,1)
rho = data(:,2)

for i = 1 : length(theat) - 1
    theat_d(i) = theat_s(i + 1) - theat_s(i);
end
theat_d

%count var
n = 1;
m = 1;

%

for i = 1 : length(theat) - 1
    if theat_d(i) < mean(theat_d)
        theat_n(n,m) = theat_s(i);
        m = m + 1;
    else
        if m ~= 1
            n = n + 1;
            m = 1;
        end
    end
end
theat_n
mean(rho)

