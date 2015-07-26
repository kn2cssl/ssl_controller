%>>1
U= [10;10;-10;-10];
for i=1:1000
U=[U [10;10;-10;-10]];
end
wave.signals.dimensions =4;
wave.signals.values = U';
t=[0:.01:10];
wave.time = t';

%>>2
