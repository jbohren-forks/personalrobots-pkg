(asdf:defsystem "decomp"
  :depends-on ("lookahead-sys")
  :components
  ((:module "dependency"
	    :components ((:file "dep-package")
			 (:file "macros" :depends-on ("dep-package"))
			 (:file "diffs" :depends-on ("macros"))
			 (:file "update-fn" :depends-on ("dep-package"))
			 (:file "dependency-graph" :depends-on ("macros" "diffs" "update-fn"))))
   (:module "valuation-bound-nodes"
	    :depends-on ("dependency")
	    :components ((:file "vb-package")
			 (:file "node" :depends-on ("vb-package"))
			 (:file "primitive" :depends-on ("node"))
			 (:file "or-node" :depends-on ("node"))
			 (:file "sequence" :depends-on ("node"))))))