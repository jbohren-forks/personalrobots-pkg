all: newmat/newmat10

newmat/newmat_1.10.4/newmat10.tar.gz:
	mkdir -p newmat/newmat_1.10.4
	cd newmat/newmat_1.10.4 && wget http://pr.willowgarage.com/downloads/newmat10.tar.gz

download: newmat/newmat_1.10.4/newmat10.tar.gz

newmat/newmat10: newmat/newmat_1.10.4/newmat10.tar.gz newmat_1.10.4-2build1.diff fPIC.diff warnings.diff
	cd newmat/newmat_1.10.4 && tar xzf newmat10.tar.gz
	cd newmat/newmat_1.10.4 && patch -p1 <../../newmat_1.10.4-2build1.diff
	cd newmat/newmat_1.10.4 && patch -p1 <../../fPIC.diff
	cd newmat/newmat_1.10.4 && patch -p0 <../../warnings.diff
	cd newmat/newmat_1.10.4 && touch config.h
	cd newmat/newmat_1.10.4 && make $(PARALLEL_JOBS) -f nm_gnu.mak
	cp -r newmat/newmat_1.10.4 newmat/newmat10

clean: 
	rm -rf newmat/newmat10

wipe: clean
	rm -rf newmat

.PHONY : clean wipe
