u = 1:32;
hold on
for i = 1:9
plot(u,grade(i).*u.^0,'-r','linewidth',2)
end
for i = 1:9
plot(grade(i).*u.^0,u,'-r','linewidth',2)
end