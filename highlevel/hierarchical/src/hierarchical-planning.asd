(asdf:defsystem "hierarchical-planning"

  :components
  ((:module "misc" :pathname "misc/"
	    :components ((:file "util-pkg")
			 (:file "macros" :depends-on ("util-pkg"))
			 (:file "utils" :depends-on ("util-pkg" "macros"))
			 (:file "clone" :depends-on ("utils"))
			 (:file "array-utils" :depends-on ("utils" "macros"))
			 (:file "list-utils" :depends-on ("utils"))
			 (:file "hash-utils" :depends-on ("utils"))
			 (:file "function-utils" :depends-on ("utils"))
			 (:file "exp-utils" :depends-on ("utils"))
			 (:file "math-utils" :depends-on ("utils"))
			 (:file "sequence-utils" :depends-on ("utils"))
			 (:file "string-utils" :depends-on ("sequence-utils"))))

   (:module "set" :pathname "data-struct/set/"
	    :depends-on ("misc")
	    :components ((:file "set")
			 (:file "seq-set" :depends-on ("set"))
			 (:file "hash-set" :depends-on ("set"))
			 (:file "number-set" :depends-on ("set"))
			 (:file "indexed-set" :depends-on ("set"))
			 (:file "recursive-enumeration" :depends-on ("set"))
			 (:file "named-sets" :depends-on ("set"))
			 (:file "inst-var-accessors" :depends-on ("set"))
			 (:file "create-sets" :depends-on ("set" "indexed-set" "inst-var-accessors"))
			 (:file "interval" :depends-on ("set"))
			 (:file "directory-set" :depends-on ("set"))
			 (:file "direct-product-set" :depends-on ("set" "inst-var-accessors"))))

   (:module "mapping" :pathname "data-struct/"
	    :depends-on ("misc" "set")
	    :components
	    ((:file "mapping")))

   (:module "data-struct" :pathname "data-struct/"
	    :depends-on ("misc" "prob" "set" "mapping")
	    :components
	    ((:file "bucketed-counts")
	     (:file "circular-vector")
	     (:file "local-search")
	     (:file "queue")
	     (:file "priority-queue")
	     (:module "graph"
	      :depends-on ("queue" "priority-queue" "union-find")
	      :components
	      ((:file "graph")
	       (:file "topological-sort" :depends-on ("graph"))
	       (:file "adjacency-list-graph" :depends-on ("graph"))
	       (:file "adjacency-matrix-graph" :depends-on ("graph"))
	       (:file "algorithms" :depends-on ("graph"))
	       (:file "connected-components" :depends-on ("graph"))
	       (:file "flow" :depends-on ("adjacency-list-graph"))))
	     (:file "skiplist")
	     (:module "tree"
	      :components ((:file "tree")
			   (:file "tree-set" :depends-on ("tree"))))
	     (:file "union-find" :depends-on ("tree"))
	     (:module "prop-logic" :depends-on ("tree")
				   :components ((:file "prop-formula")
						(:file "dnf" :depends-on ("prop-formula"))
						(:file "dnf-set" :depends-on ("dnf"))))))
			   


   (:module "math" :pathname "math/" :depends-on ("misc" "set")
	    :components ((:file "lin-alg")
			 (:file "chi-square")
			 (:file "svd" :depends-on ("lin-alg"))
			 ))



   (:module "prob"
	    :depends-on ("set" "mapping" "math")
	    :components ((:file "probability-distribution")
			 (:file "function-random-variable" :depends-on ("probability-distribution"))
			 (:file "information-theory" :depends-on ("probability-distribution"))
			 (:file "create-distributions" :depends-on ("probability-distribution"))
			 (:file "vector-probability-distribution" :depends-on ("probability-distribution"))
			 (:file "hash-table-prob-dist" :depends-on ("probability-distribution"))
			 (:module "parametric"
			  :depends-on ("probability-distribution")
			  :components ((:file "uniform")
				       (:file "gaussian")))
			 (:file "alist-probability-distribution" :depends-on ("probability-distribution"))))

   (:module "geometry"
	    :depends-on ("math" "prob") :pathname "math/geometry/"
	    :components
	    ((:file "geometry")
	     (:module "transform" :depends-on ("geometry")
				  :components
				  ((:file "transformation")
				   (:file "rigid-2d" :depends-on ("transformation"))))
	     (:module "point-sets" :depends-on ("transform")
				   :components
				   ((:file "point-sets")
				    (:file "discrete" :depends-on ("point-sets"))
				    (:file "sphere" :depends-on ("point-sets"))
				    (:file "polygon" :depends-on ("point-sets" "sphere"))
				    ))
	     (:module "conf-sets" :depends-on ("point-sets" "transform")
				  :components
				  ((:file "conf-set")
				   (:file "rigid-2d-motions" :depends-on ("conf-set"))))
	     (:file "geom-language" :depends-on ("transform" "point-sets"))
	     (:module "algorithms" :depends-on ("point-sets" "transform")
				   :components
				   ((:file "general")))))

		 
   (:module "env" :depends-on ("misc" "set" "prob")
	    :components ((:file "env-pkg")
			 (:file "env" :depends-on ("env-pkg"))
			 (:file "fully-observable-env" :depends-on ("env"))
			 (:file "trajectory" :depends-on ("env"))))


   (:module "angelic"
	    :depends-on ("misc" "set" "data-struct" "mapping" "env")
	    :components
	    ((:file "angelic-pkg")
	     (:file "planning-problem" :depends-on ("angelic-pkg"))
	     (:file "hierarchy" :depends-on ("angelic-pkg" "planning-problem"))
	     (:file "description" :depends-on ("angelic-pkg"))
	     (:file "variable-hierarchy" :depends-on ("description" "hierarchy"))
	     (:file "abstract-planning-problem" :depends-on ("planning-problem" "hierarchy" "description"))
	     (:file "subsumption" :depends-on ("description"))
	     (:module "lookahead" :depends-on ("abstract-planning-problem" "subsumption" "angelic-pkg")
				  :components
				  ((:file "abstract-lookahead-tree")
				   (:file "nodes" :depends-on ("abstract-lookahead-tree"))
				   (:file "meta" :depends-on ("nodes"))
				   (:file "hrt" :depends-on ("abstract-lookahead-tree"))
				   (:file "ahrta" :depends-on ("abstract-lookahead-tree"))
				   (:file "offline" :depends-on ("nodes"))))
	     (:module "prop"
	      :depends-on ("angelic-pkg" "lookahead")
	      :components
	      ((:file "csp")
	       (:file "prop-domain" :depends-on ("csp"))
	       (:file "ncstrips" :depends-on ("prop-domain"))
	       (:file "prop-hierarchy" :depends-on ("prop-domain"))
	       (:file "prop-abstract-planning-problem" :depends-on ("prop-domain"))))))

   (:module "decomp"
	    :depends-on ("angelic")
	    :components
	    ((:file "decomp")
	     (:module "dependency"
		      :components ((:file "dep-package")
				   (:file "macros" :depends-on ("dep-package"))
				   (:file "diffs" :depends-on ("macros"))
				   (:file "update-fn" :depends-on ("dep-package"))
				   (:file "dependency-graph" :depends-on ("macros" "diffs" "update-fn"))))
	     (:module "valuation-bounds"
		      :depends-on ("dependency" "decomp")
		      :components ((:file "vb-package")
				   (:file "node" :depends-on ("vb-package"))
				   (:file "primitive" :depends-on ("node"))
				   (:file "or-node" :depends-on ("node"))
				   (:file "descriptions" :depends-on ("node"))
				   ))))
					    
   (:module "envs" :depends-on ("angelic" "motion-planning" "hybrid" "decomp")
	    :components
	    ((:file "grid-world")
	     (:module "blocks"
	      :components ((:file "blocks-ceiling")
			   (:file "state-set" :depends-on ("blocks-ceiling"))
			   (:file "hierarchy" :depends-on ("state-set"))
			   (:file "decomp-hierarchy" :depends-on ("state-set"))
			   (:file "descriptions" :depends-on ("state-set"))
			   (:file "decomp-descriptions" :depends-on ("decomp-hierarchy" "descriptions"))
			   (:file "subsumption" :depends-on ("state-set"))))
	     (:module "nav-switch" 
	      :components ((:file "nav-switch")
			   (:file "hierarchy" :depends-on ("nav-switch"))
			   (:file "descriptions" :depends-on ("nav-switch"))))
							      
	     (:module "pick-place" 
	      :components 
	      ((:file "pick-place")
	       (:file "state-set" :depends-on ("pick-place"))))))

		 
   (:module "motion-planning" :depends-on ("math" "prob" "geometry")
	    :components ((:file "motion-planning")
			 (:module "cspace"
			  :depends-on ("motion-planning")
			  :components
			  ((:file "cspace")
			   (:file "simple-cs" :depends-on ("cspace"))
			   (:file "cspace-family" :depends-on ("simple-cs"))))
			 (:module "geometric"
			  :depends-on ("motion-planning")
			  :components
			  ((:file "visibility")))
			 (:module "roadmap"
			  :depends-on ("cspace")
			  :components 
			  ((:file "roadmap")
			   (:file "visibility" :depends-on ("roadmap"))
			   (:file "simple" :depends-on ("roadmap"))))))
		 
		      

	
	 

   (:module "hybrid" :depends-on ("angelic" "motion-planning") :pathname "motion-planning/"
	    :components
	    ((:module "hybrid" :components ((:file "hybrid-domain")))
	     (:module "roadmap" :components ((:file "hybrid")) :depends-on ("hybrid"))))))


	
(in-package cl-user)



;; Local variables:
;; mode:lisp
;; outline-regexp:"\\s-*..module"
;; End:
