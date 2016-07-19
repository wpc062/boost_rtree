#!/bin/bash
set -x
g++ --version | head -1
echo "generate with boost 1.59..."
g++ -I/home/pwang/local/include ../rtree_test.cc -o rtree_test.O2.packing -O2
g++ -I/home/pwang/local/include ../rtree_test.cc -o rtree_test.packing
g++ -I/home/pwang/local/include ../rtree_test.cc -o rtree_test.O2 -O2 -DNO_PACK_ALGO
g++ -I/home/pwang/local/include ../rtree_test.cc -o rtree_test -DNO_PACK_ALGO

# echo "generate with boost 1.57..."
# g++ -I/usr/local/brion/boost/1.57.0_1/include ../rtree_test.cc -o rtree_test.57.O2.packing -O2
# g++ -I/usr/local/brion/boost/1.57.0_1/include ../rtree_test.cc -o rtree_test.57.packing
# g++ -I/usr/local/brion/boost/1.57.0_1/include ../rtree_test.cc -o rtree_test.57.O2 -O2 -DNO_PACK_ALGO
# g++ -I/usr/local/brion/boost/1.57.0_1/include ../rtree_test.cc -o rtree_test.57 -DNO_PACK_ALGO
