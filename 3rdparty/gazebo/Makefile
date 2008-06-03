all: gazebo

.PHONY: all gazebo clean wipe
gazebo-git:
	@if [ ! -d gazebo ]; then git clone /u/pr/simpr/pr2sim.git gazebo-git; fi

gazebo: gazebo-git
	(cd gazebo-git && /bin/bash -c "source setup.bash; make")

clean:
	-cd gazebo-git && make clean

wipe:
	rm -rf gazebo-git

