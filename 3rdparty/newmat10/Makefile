all: newmat

newmat:
	mkdir -p newmat/newmat_1.10.4
	cd newmat/newmat_1.10.4 && wget http://www.robertnz.net/ftp/newmat10.tar.gz
	cd newmat/newmat_1.10.4 && tar xzf newmat10.tar.gz
	cd newmat/newmat_1.10.4 && patch -p1 <../newmat_1.10.4-2build1.diff

clean: 
	rm -rf newmat

.PHONY : clean
