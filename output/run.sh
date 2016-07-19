#!/bin/sh
#set -x
if [[ -z "$1" ]]; then
    echo "Usage: $0 <file name>"
    exit 1
fi

#bin_list=(`ls rtree_test*O2*`) #array
bin_list=`ls rtree_test*` #string
#echo "binary list: ${bin_list[@]}, len=${#bin_list[@]}"
for bin in  $bin_list
do
    a=$(file $bin | grep ELF)
    if [[ -z "$a" ]]; then
        continue
    fi

    if [[ ! -x "$bin" ]] ; then
        continue
        :
    fi
    echo "testing ./$bin $1"
    (exec "./$bin" "$1")
    #echo "next..."
done

