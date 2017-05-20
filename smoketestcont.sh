#!/bin/bash

f=../recyclingrobots/simRR.txt
n=3
r=1
i=7
k=1
t=2
s=10
a=0.1
er=1000
ed=150
d=0.9

echo ./runcgdice -f $f -n $n -r $r -i $i -k $k -t $t -s $s -a $a -er $er -ed $ed -d $d -w smpl.dist
./runcgdice -f $f -n $n -r $r -i $i -k $k -t $t -s $s -a $a -er $er -ed $ed -d $d -w smpl.dist
