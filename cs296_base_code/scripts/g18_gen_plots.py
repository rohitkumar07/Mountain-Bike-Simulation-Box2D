#! /usr/bin/env python3

import random
import numpy as np
import matplotlib.pyplot as plt
from pylab import *

mycsv = open("./data/g18_lab09data.csv",'r');

itr_range = 100;
reruns = 25;

step_val = [];
coll_val = [];
velo_val = [];
pos_val = [];
sum_val = [];
loop_val = [];


for x in range(itr_range):
	step_val.append(0);
	coll_val.append(0);
	velo_val.append(0);
	loop_val.append(0);
	pos_val.append(0);
	sum_val.append(0);

for line in mycsv:
	splited = line.split();
	coll_val[int(splited[0])-1] = coll_val[int(splited[0])-1] + float(splited[3]);
	loop_val[int(splited[0])-1] = loop_val[int(splited[0])-1] + float(splited[6]);
	step_val[int(splited[0])-1] = step_val[int(splited[0])-1] + float(splited[2]);
	velo_val[int(splited[0])-1] = velo_val[int(splited[0])-1] + float(splited[4]);
	pos_val[int(splited[0])-1] = pos_val[int(splited[0])-1] + float(splited[5]);

for i in range(itr_range):
	coll_val[i]=coll_val[i]/reruns;
	step_val[i]=step_val[i]/reruns;
	velo_val[i]=velo_val[i]/reruns;
	loop_val[i]=loop_val[i]/reruns;
	pos_val[i]=pos_val[i]/reruns;

for i in range(itr_range):
	sum_val[i] = coll_val[i]+step_val[i]+velo_val[i]+pos_val[i];

#####################################drawing graph 1 ##################	
	
max_loop = max(loop_val);
min_loop = min(loop_val);

max_step = max(step_val);
min_step = min(step_val);

def graph(formula, x_range,c):  
    x = np.array(x_range)  
    y = eval(formula)
    plt.plot(x, y,color=c)

ind = np.arange(itr_range)  # the x locations for the groups
width = 0.25       # the width of the bars

fig, ax = plt.subplots()
rects_loop = ax.bar(ind, loop_val, width, color='r', label = 'loop time')
rects_step = ax.bar(ind+width, step_val, width, color='y', label = 'step time')
#graph('max_loop+x*0', range(1,50),'b')
x = np.array(range(1,itr_range));
y = eval('max_loop+x*0');
max_loop_draw = plt.plot(x,y,color = 'c', label = 'max loop time')

#graph('min_loop+x*0', range(1,50),'g')
x = np.array(range(1,itr_range));
y = eval('min_loop+x*0');
min_loop_draw = plt.plot(x,y,color = 'g',label = 'min loop time')

#graph('max_step+x*0', range(1,50),'m')
x = np.array(range(1,itr_range));
y = eval('max_step+x*0');
max_step_draw = plt.plot(x,y,color = 'm', label = 'max step time')

#graph('min_step+x*0', range(1,50),'k')
x = np.array(range(1,itr_range));
y = eval('min_step+x*0');
min_step_draw = plt.plot(x,y,color = 'c', label = 'min step time')

ax.set_ylabel('step/loop time')
ax.set_xlabel('Iteration')
ax.set_title('Time vs iteration')
ax.legend(loc=2)

fig.savefig('./plots/g18_lab09_plot01.png')
plt.close(fig)
	
#####################################drawing graphs 2##################	
ind = np.arange(itr_range)  # the x locations for the groups
width = 0.1       # the width of the bars

fig, ax = plt.subplots()
x = np.arange(1,itr_range+1);
y1 = np.array(coll_val);
plt.plot(x,y1,label = 'collision time', color = 'r')

y2 = np.array(step_val);
plt.plot(x,y2,label = 'step time', color = 'b')

y3 = np.array(velo_val);
plt.plot(x,y3,label = 'velocity time', color = 'g')

y4 = np.array(pos_val);
plt.plot(x,y4,label = 'position update time', color = 'c')

y5 = np.array(sum_val);
plt.plot(x,y5,label = 'total time', color = 'k')
ax.set_xlabel('Iteration')
ax.set_ylabel('step/collision/velocity/position update time')
ax.set_title('Time vs iteration')
ax.legend(loc=0)
#plt.show()
fig.savefig('./plots/g18_lab09_plot02.png')
plt.close(fig)
############################ graph 3 ###################################

mycsv = open("./data/g18_lab09data.csv",'r');

deviation=[];
temp = [];
for line in mycsv:
	splited = line.split();
	temp.append(float(splited[2]));
	if(len(temp)==reruns):
		deviation.append(np.std(temp));
		temp=[];
		
fig, ax = plt.subplots()
y = np.array(step_val)
step_time = plt.errorbar(x, y,color='r', label = 'step time', yerr = deviation);
ax.set_xlabel('Iteration')
ax.set_ylabel('step time')
ax.set_title('Step Time with error bars vs iteration')
ax.legend(loc=0)
#plt.show()
fig.savefig('./plots/g18_lab09_plot03.png')
plt.close(fig)

#############################graph 4 #####################################

mycsv = open("./data/g18_lab09data.csv",'r');
highest_roll = 35;
count_parts = 200;
frequency = [];
for i in range(count_parts+1):
	frequency.append(0);

temp = [];
found = False;
for line in mycsv:
	splited = line.split();
	if(int(splited[0]) == highest_roll):
		temp.append(float(splited[2]));
		continue;
	if(found):
		break;

min_elt = min(temp);
max_elt = max(temp);
#print(max_elt)
unit = (max_elt-min_elt)/count_parts;

for i in temp:
	for j in (range(count_parts+1)):
		if(( i >= min_elt + j*unit) and (i < min_elt + (j+1)*unit)):
			frequency[j] = frequency[j]+1;
			break;
#print(frequency[:], len(frequency))
cumulative = [frequency[0]];
for i in range(1,len(frequency)):
	cumulative.append(frequency[i]+cumulative[i-1]);
#	print(cumulative[i])
#print(cumulative[:],len(cumulative))
#print(len(np.arange(min_elt,max_elt+unit,unit)));

fig, ax = plt.subplots()
ax.bar(np.arange(min_elt,max_elt+unit/2,unit), frequency, unit , color='r', label = 'frequency');
plt.plot(np.arange(min_elt,max_elt+unit/2,unit),np.array(cumulative),label = 'cumulative', color = 'k');
ax.set_xlabel('Step time')
ax.set_ylabel('No of reruns')
ax.set_title('Frequency and cummulative frequency plot')
ax.legend(loc=0)
#plt.show()
fig.savefig('./plots/g18_lab09_plot04.png')
plt.close(fig)
mycsv.close()
########################## graph 5##############################

import random

countToSelect = 10;  ## count for random numbers in the plot

mycs = open("./data/g18_lab09data.csv",'r');

rand_average=[]
temp_array = []
for line in mycs:
	splited = line.split();
	temp_array.append(float(splited[2]))
	if(len(temp_array) == reruns):			
		my_randoms = random.sample(range(0, reruns), countToSelect)
		random_array = []
		for i in range(countToSelect):
			random_array.append(temp_array[my_randoms[i]]);
		rand_average.append((sum(random_array))/countToSelect);
		temp_array=[];

fig, ax = plt.subplots()
x = [i for i in range(1,itr_range+1)];
plt.plot(x,step_val,'ro',color = 'r', label = 'step times');
plt.plot(x,rand_average,'ro',color = 'b', label = 'random step times over reruns');

fit = polyfit(x,step_val,1)
fit_fn = poly1d(fit) # fit_fn is now a function which takes in x and returns an estimate for y
plot(x, step_val, 'ro', x, fit_fn(x), '--r', label = 'best fit over all points')

fit1 = polyfit(x,rand_average,1)
fit_fn1 = poly1d(fit1) # fit_fn is now a function which takes in x and returns an estimate for y
plt.plot(x,rand_average, 'bo', x, fit_fn1(x), '--b', label = 'best fit over random points')
ax.set_xlabel('iteration')
ax.set_ylabel('avg step time and step time over random reruns')
ax.set_title('Best Fit plots')
ax.legend(loc=0)

fig.savefig('./plots/g18_lab09_plot05.png')
plt.close(fig)
#################################################################

