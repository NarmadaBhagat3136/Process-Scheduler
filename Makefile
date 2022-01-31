SHELL:=/bin/bash
	CPPFLAGS=-static -g
	CC=gcc-9.1
	CPP=g++-9.1
sched:scheduler.cc
	(module unload $(CC);\
	module load $(CC);\
	$(CPP) $(CPPFLAGS) -o sched scheduler.cc)
	
clean:
	rm -f sched  

