all: gazebo

gazebo-git:
	@if [ ! -d gazebo ]; then git clone /u/pr/simpr/pr2sim.git gazebo-git; fi

gazebo: gazebo-git
	cd gazebo-git && /bin/bash -c "source setup.bash; cd packages; make"
	cd gazebo-git && /bin/bash -c "source setup.bash; cd plugin; scons; scons install"
	cd gazebo-git && /bin/bash -c "source setup.bash; cd examples/pr2API; scons; scons install"

clean:
	-cd gazebo-git && make clean

wipe:
	rm -rf gazebo-git

