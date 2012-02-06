

#!/bin/bash


arg1=$1
arg2=$2

#img_path="~/image_ln"

function initfunc()
{

product=$arg1
if [[ $arg1 = [-][hH] ]];then
	echo "STM912HC"
	echo "STM803HC"
	echo "STM722HC"
	echo "STM722HZ"
	echo "STM722HCZ"
	echo "STM908HC"
	echo "STM908HDZ"

	
	exit 0
fi
init=y
if [[ "$arg2" = [nN] ]];then
	init=N
fi
img_path="/home/mgq/image_ln"


}


function check_exit()
{
	if [ $? != 0 ]
	then
		echo "error: return value. "
		exit 1  #$?
	fi
}

function cp_img()
{
	cp kernel.img $img_path 
}

###### main ##########

initfunc

echo "$init"

if [ "$product" = "" ];then
	echo "usage error"
	exit 1
elif [[ "$product" = [--][hH] ]];then
	echo "m901hr [y|Y]"
	
	exit 0
elif [[ "$init" = [yY] ]];then
	if [[ $product = [mM]901[hH][rR] ]];then
		echo "make M901HR_defconfig ....."
		make M901HR_defconfig
		check_exit
		echo "make M901HR_defconfig finish!"
	elif [[ $product = [mM]732[hH][cC] ]];then
		echo "make M732HC_defconfig ....."
		make M732HC_defconfig
		check_exit
		echo "make M732HC_defconfig finish!"
	elif [[ $product = [sS][tT][mM]912[hH][cC] ]];then
		echo "make STM912HC_defconfig ....."
		make STM912HC_defconfig
		check_exit
		echo "make STM912HC_defconfig finish!"
	elif [[ $product = [sS][tT][mM]803[hH][cC] ]];then
		echo "make M803HC_defconfig ....."
		make M803HC_defconfig
		check_exit
		echo "make M803HC_defconfig finish!"
	elif [[ $product = [sS][tT][mM]722[hH][cC] ]];then
		echo "make M722HC_defconfig ....."
		make M722HC_defconfig
		check_exit
		echo "make M722HC_defconfig finish!"
	elif [[ $product = [sS][tT][mM]722[hH][cC][zZ] ]];then
		echo "make M722HCZ_defconfig ....."
		make M722HC_defconfig
		check_exit
		echo "make M722HCZ_defconfig finish!"
	elif [[ $product = [sS][tT][mM]722[hH][zZ] ]];then
		echo "make M722HZ_defconfig ....."
		make M722HZ_defconfig
		check_exit
		echo "make M722HZ_defconfig finish!"
	elif [[ $product = [sS][tT][mM]908[hH][cC] ]];then
		echo "make M908HC_defconfig ....."
		make M908HC_defconfig
		check_exit
		echo "make M908HC_defconfig finish!"
	elif [[ $product = [sS][tT][mM]908[hH][dD][zZ] ]];then
		echo "make M908HD_defconfig ....."
		make M908HD_defconfig
		check_exit
		echo "make M908HD_defconfig finish!"
	



	#### add branch at here #########
	#  elif [   ];then              #
	#				#
	#################################
	else
		echo "error: no product!"
		exit 1
	fi
	
fi

echo "make kernel.img ......."
make kernel.img
check_exit
echo "make kernel.img finish!"
echo "cp kernel.img to  "$img_path" of xp ....."
cp_img
check_exit
echo "cp kernel.img to  "$img_path" of xp finish!"

############## end #########################




