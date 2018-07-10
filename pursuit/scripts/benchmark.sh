#!/bin/bash
params=(0.1000 0.1395 0.1946 0.2714 0.3786 0.5282 0.7368 1.0278 1.4337 2.0000)
len=${#params[*]}

for (( i=0; i<len; i++ ))
do
	node_density=${params[$i]}

	mkdir output$i

	printf "node_density: $node_density \nnobs: 20 \nnp: 3 \n" 
	printf "node_density: $node_density \nnobs: 20 \nnp: 3 \n" > output$i/readme.txt

	for number in {1..100}
	do
		echo "Executing map $number "
		rosrun pursuit benchmark $node_density $number  > output$i/map$number.csv
	done
done

exit 0