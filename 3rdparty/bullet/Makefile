all: installed

SVN_DIR = bullet_svn
SVN_URL = http://bullet.googlecode.com/svn/trunk/
SVN_PATCH = quaternion.patch demos.patch
SVN_REVISION=-r1555

include $(shell rospack find mk)/svn_checkout.mk

.PHONY: build


# Poor man's installation procedure setup
BULLET_LIBS = $(SVN_DIR)/src/BulletCollision/libBulletCollision.a \
              $(SVN_DIR)/src/BulletDynamics/libBulletDynamics.a \
              $(SVN_DIR)/src/BulletSoftBody/libBulletSoftBody.a \
              $(SVN_DIR)/src/LinearMath/libLinearMath.a

BULLET_INC_DIRS = BulletSoftBody \
                  BulletCollision/CollisionShapes \
                  BulletCollision/BroadphaseCollision \
                  BulletCollision/NarrowPhaseCollision \
                  BulletCollision/CollisionDispatch \
                  BulletCollision/Gimpact \
                  BulletDynamics/ConstraintSolver \
                  BulletDynamics/Vehicle \
                  BulletDynamics/Dynamics \
                  BulletDynamics/Character \
                  LinearMath \

installed: $(SVN_DIR) patched
	cd $(SVN_DIR) && cmake -DCMAKE_INSTALL_PREFIX=$(PWD) .
	# Bullet appears not be parallel-make safe
	#cd $(SVN_DIR) && make $(PARALLEL_JOBS) 
	cd $(SVN_DIR) && make
	# The 'install' target only works with cmake 2.6 for some reason
	#cd $(SVN_DIR) && make $(PARALLEL_JOBS) install
	mkdir -p lib
	cp $(BULLET_LIBS) lib
	mkdir -p include
	$(foreach d,$(BULLET_INC_DIRS), mkdir -p include/$(d) && cp $(SVN_DIR)/src/$(d)/*.h include/$(d);)
	touch installed

clean:
	rm -f installed
	-make -C $(SVN_DIR) clean

wipe: clean
	-rm -f patched
	-rm -rf bullet_svn
	-rm -rf build
