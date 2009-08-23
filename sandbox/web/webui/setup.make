all: db/webui/webui.db3 db/auth/auth.db3

db/webui/webui.db3: 
	(cd src/webui/mod/webui/; ./createdb.py)

db/auth/auth.db3:
	(cd src/webui/auth/; ./createdb.py)

