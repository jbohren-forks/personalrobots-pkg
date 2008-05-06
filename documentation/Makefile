all: docs
docs: 
	python doxymaker.py

upload: docs
	scp -P 2222 -r doc /var/www/pr.willowgarage.com/html/pr-docs/personalrobots

upload-local: docs
	rm -rf /var/www/pr.willowgarage.com/html/pr-docs/personalrobots
	cp -a doc /var/www/pr.willowgarage.com/html/pr-docs/personalrobots

clean: 
	rm -rf doc
