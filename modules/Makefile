GCC = g++

MODULES = ctrl_module sim_module

all: $(MODULES)
ctrl_module: ctrl_module.o  arm.o  
	$(GCC)   -o $@ $<   arm.o  \
	-L/usr/local/lib -L/usr/lib  -lktl -lm -lpthread -lts01

sim_module: sim_module.o arm.o
	$(GCC)   -o $@ $< arm.o \
	-L/usr/local/lib -L/usr/lib -lktl -lts01 -lm -lpthread

ctrl_module.o: ctrl_module.cpp controller.h arm.h
	$(GCC) -W ${INCLUDE} ${CXXFLAGS} -c $< -o $@ 

sim_module.o: sim_module.cpp sim.h arm.h
	$(GCC) -W ${INCLUDE} ${CXXFLAGS} -c $< -o $@

arm.o: arm.cpp arm.h
	$(GCC) -W ${INCLUDE} ${CXXFLAGS} -c $< -o $@ 

clean:
	rm -f $(MODULES) *.o  *~


