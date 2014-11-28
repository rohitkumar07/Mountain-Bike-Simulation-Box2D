#! /usr/bin/env python3

from subprocess import check_output

executable = "./mybins/cs296_18_exe"

temp = open('./data/g18_lab09data.csv','w')


for i in range(1,101) :
	for j in range(1,26) :
		out = check_output([executable, str(i)])
		res = str(out).split()
		temp.write(str(i) + "\t")
		temp.write(str(j) + "\t")
		while int(float(res[8])) > 1000 :
			out = check_output([executable, str(i)])
			res = str(out).split()
		temp.write(res[8] + "\t")
		temp.write(res[14] + "\t")
		temp.write(res[21] + "\t")
		temp.write(res[28] + "\t")
		temp.write(res[33] + "\n")
	print("currently in itr. " + str(i))


