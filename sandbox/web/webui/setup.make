all: db/webui/webui.db3 db/auth/auth.db3

db/webui/webui.db3: 
	(cd src/webui/mod/webui/; ./db_webui.py)

db/auth/auth.db3:
	(cd src/webui/auth/; ./db_auth.py)

