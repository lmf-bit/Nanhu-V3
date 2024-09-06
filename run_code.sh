#!/bin/bash
export NOOP_HOME=$(pwd)
#git restore build.sc

make simv RUN_BIN=linux.bin
make simv-run RUN_BIN=linux.bin
#make simv RUN_BIN=microbench.bin
#make simv-run RUN_BIN=microbench.bin

#make simv RUN_BIN=coremark-3-iteration.bin
#make simv-run RUN_BIN=coremark-3-iteration.bin

#make emu_rt -j32
#make emu_rtl-run RUN_BIN=microbench.bin

date
date > date.log
