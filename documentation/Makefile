all: docs
docs: 
	doxygen doxy

upload: docs
	scp -P 2222 -r html /var/www/pr.willowgarage.com/html/pr-docs/personalrobots

upload-local: docs
	cp -R html /var/www/pr.willowgarage.com/html/pr-docs/personalrobots

clean: 
	rm -rf html latex
