(asdf:defsystem "hierarchical"

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
		      :depends-on ("dependency")
		      :components ((:file "vb-package")
				   (:file "node" :depends-on ("vb-package"))
				   ;;(:file "primitive" :depends-on ("node"))
				   ;;(:file "or-node" :depends-on ("node"))
				   ;;(:file "sequence" :depends-on ("node"))
				   ))))
					    
   (:module "envs" :depends-on ("angelic" "motion-planning" "hybrid" "decomp")
	    :components
	    ((:file "grid-world")
	     (:module "blocks"
	      :components ((:file "blocks-ceiling")
			   (:file "hierarchy" :depends-on ("blocks-ceiling"))
			   (:file "state-set" :depends-on ("blocks-ceiling"))
			   (:file "descriptions" :depends-on ("blocks-ceiling" "state-set"))
			   (:file "decomp-hierarchy" :depends-on ("state-set" "blocks-ceiling"))
			   (:file "subsumption" :depends-on ("blocks-ceiling"))))
	     (:module "nav-switch" 
	      :components ((:file "nav-switch")
			   (:file "hierarchy")
			   (:file "descriptions")))
							      
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



#|


(asdf:defsystem "lookahead-sys"
    :depends-on ("hrl")
    :components ((:module "lookahead-mod" :pathname "lookahead/"
			  :components
			  ((:file "lookahead")
			   (:file "planning-problem" :depends-on ("lookahead"))
			   (:file "hierarchy" :depends-on ("lookahead" "planning-problem"))
			   (:file "description" :depends-on ("lookahead"))
			   (:file "variable-hierarchy" :depends-on ("description" "hierarchy"))e
			   (:file "abstract-planning-problem" :depends-on ("planning-problem" "hierarchy" "description"))
			   (:file "subsumption" :depends-on ("description")) 
			   (:module "lookahead-mod2" :pathname "lookahead/" :depends-on ("abstract-planning-problem" "subsumption")
				    :components
				    ((:file "abstract-lookahead-tree")
				     (:file "nodes" :depends-on ("abstract-lookahead-tree"))
				     (:file "meta" :depends-on ("nodes"))
				     (:file "hrt" :depends-on ("abstract-lookahead-tree"))
				     (:file "ahrta" :depends-on ("abstract-lookahead-tree"))
				     (:file "offline" :depends-on ("nodes"))))
			   (:module "prop"
				    :depends-on ("lookahead-mod2")
				    :components
				    ((:file "csp")
				     (:file "prop-domain" :depends-on ("csp"))
				     (:file "ncstrips" :depends-on ("prop-domain"))
				     (:file "prop-hierarchy" :depends-on ("prop-domain"))
				     (:file "prop-abstract-planning-problem" :depends-on ("prop-domain"))))))
					    
		 (:module "envs" :depends-on ("lookahead-mod" "motion-planning")
			  :components
			  ((:module "blocks"
				    :components ((:file "blocks-ceiling")
						 (:file "hierarchy" :depends-on ("blocks-ceiling"))
						 (:file "descriptions" :depends-on ("blocks-ceiling"))
						 (:file "subsumption" :depends-on ("blocks-ceiling"))))
			   (:module "driverlog" :components ((:file "driverlog")))
			   (:module "nav-switch" 
				    :components ((:file "nav-switch")
						 (:file "hierarchy")
						 (:file "descriptions")))
							      
			   (:module "depots" 
				    :components ((:file "depots")
						 (:file "hierarchy" :depends-on ("depots"))
						 (:file "descriptions" :depends-on ("depots"))))
			   (:module "pick-place" 
				    :components 
				    ((:file "pick-place")
				     (:file "state-set" :depends-on ("pick-place"))
				     (:file "hierarchy" :depends-on ("pick-place"))
				     (:file "descriptions" :depends-on ("pick-place"))))
			   (:module "forage" :components ((:file "forage")))))
		 
		 (:module "motion-planning" :depends-on ("lookahead-mod")
			  :components
			  ((:module "hybrid" :components ((:file "hybrid-domain")))
			   (:module "roadmap" :components ((:file "hybrid")))))))


   (:module "fn-approx" :pathname "fn-approx/"
	    :depends-on ("misc")
	    :components ((:file "fn-approx")
			 (:file "tabular-fn-approx" :depends-on ("fn-approx"))
			 (:file "linear-fn-approx" :depends-on ("fn-approx"))))

   (:module "rl-functions"
	    :depends-on ("fn-approx" "misc" "data-struct" "prob" "math" "mdp")
	    :components ((:module "value-function" :depends-on ("policy")
						   :components ((:file "value-function")
								(:file "tabular-value-function" :depends-on ("value-function"))))
			 
			 (:module "q-function"
			  :depends-on ("value-function" "policy")
			  :components ((:file "q-function")
				       (:module "crl" :depends-on ("q-function")
						      :components
						      ((:file "crl-q-function")
						       (:file "crl-features"
							:depends-on 
							("crl-q-function"))))
				       (:file "sum-q-function" 
					:depends-on ("q-function"))
				       (:file "approx-q-function" :depends-on 
					      ("q-function"))
				       (:file "tabular-q-function" :depends-on
					      ("q-function"))
				       (:module "decomposed" :depends-on ("crl")
							     :components
							     ((:file "decomposed-q-function")
							      (:file "decomposed-crl-q-function")
							      (:file "decomposed-tabular-q-function")))
				       (:file "env-q-function" :depends-on 
					      ("approx-q-function"))
				       ))
				       
			 (:module "policy"
			  :components ((:file "policy")
							     
				       (:file "random-policy" :depends-on ("policy"))
				       (:file "tabular-policy" :depends-on ("policy"))
				       (:file "prompt-policy" :depends-on ("policy"))
				       ))
				       
			 (:module "policy2" :depends-on ("q-function" "policy")
					    :pathname "policy/"
					    :components ((:file "greedy-policy")
							 (:module "exp-pol"
							  :components
							  ((:file "exploration-policy")))))
				       

			 (:module "learning-rate"
			  :components ((:file "learning-rate")
				       (:file "polynomial-learning-rate" 
					:depends-on ("learning-rate"))))))

   (:module "mdp" :depends-on ("env" "data-struct" "math" "prob" "misc")
	    :in-order-to ((compile-op (load-op "misc")))						   
	    :components ((:file "mdp-pkg")
			 (:file "smdp"
			  :depends-on ("mdp-pkg"))
			 (:file "mdp"
			  :depends-on ("smdp"))
			 (:file "mdp-env"
			  :depends-on ("mdp"))
			 (:file "2tbn-mdp-env"
			  :depends-on ("mdp-env"))
			 (:file "tabular-smdp"
			  :depends-on ("smdp"))
			 (:file "hierarchical-smdp"
			  :depends-on ("smdp" "tabular-smdp"))
			 (:file "tabular-mdp" :depends-on ("mdp"))
			 (:file "sparse-mdp" :depends-on ("mdp"))))
     
   (:module "dp" :depends-on ("mdp" "data-struct" "misc" "math" "rl-functions")
	    :components ((:file "dp")
			 (:file "mdp-dp" :depends-on ("dp"))
			 (:file "sparse-dp" :depends-on ("mdp-dp"))
			 (:file "hierarchical-dp" :depends-on ("dp" "sparse-dp"))
			 (:file "markov-chain" :depends-on ("dp"))
			 ))
     
   (:module "rl" :depends-on ("mdp" "data-struct" "misc" "env" "rl-functions" "dp")
	    :components ((:file "reinforcement-learning")
			 (:file "rl-observer" :depends-on ("reinforcement-learning"))
			 (:file "rl-control" :depends-on ("rl-observer" "reinforcement-learning"))
			 (:file "rl-user" :depends-on ("reinforcement-learning" "rl-observer" 
										"rl-control" "obs"))
			 (:module "obs"
			  :depends-on ("rl-observer")
			  :components ((:file "progress-printer")
				       (:file "stat-gatherer")
				       (:file "message-logger")
				       (:file "trajectory-gatherer")
				       (:file "env-observer")))
			 (:module "learn"
			  :depends-on ("obs")
			  :components ((:file "learning-algorithm")
				       (:file "q-learning" :depends-on ("learning-algorithm"))
				       (:file "advantage-updating" :depends-on ("learning-algorithm"))
				       (:file "decomposed-advantage-updating"
					:depends-on ("advantage-updating"))
				       (:file "decomposed-q-learning" :depends-on
					      ("q-learning"))
				       (:file "approximate-policy-iteration"
					:depends-on ("learning-algorithm"))
				       (:file "gold-standard" :depends-on ("learning-algorithm"))
				       (:file "certainty-equivalence" :depends-on ("learning-algorithm"))
				       ))))
		 
   (:module "boltzmann-exploration" :depends-on ("rl" "rl-functions" "prob")
	    :pathname "rl-functions/policy/exp-pol/"
	    :components ((:file "boltzmann-exp-pol")
			 (:file "epsilon-boltzmann-exp-pol" :depends-on ("boltzmann-exp-pol"))))


		 
   ))
     
			    

		 


		 
		 (:module "alisp" :in-order-to ((compile-op (load-op "misc")))
			  :depends-on ("misc" "env" "data-struct" "rl-functions" "rl" "dp")
			  :components ((:file "alisp")
				       (:file "alisp-state" :depends-on ("alisp"))
				       (:file "alisp-observer" :depends-on ("alisp"))
				       (:file "alisp-program" :depends-on ("alisp"))
				       (:file "rlm" :depends-on ("alisp" "alisp-observer" "alisp-program" "alisp-state"))
				       (:file "alisp-user" :depends-on ("alisp" "rlm" "alisp-program" 
										"obs" "alisp-observer"))
				       
				       (:module "rl-functions" :depends-on ("alisp" "alisp-state")
						:components ((:file "alisp-approx-q-function")
							     (:file "alisp-features")
							     (:file "exit-distribution")
							     (:file "array-exit-distribution"
								    :depends-on ("exit-distribution"))))
					      
				       (:module "obs"
						:depends-on ("alisp" "alisp-observer")
						:components ((:file "alisp-io-int-observer")
							     (:file "progress-printer")))
				       
				       (:module "learn"
						:depends-on ("alisp" "alisp-observer" "rl-functions")
						:components ((:file "learning-algorithm")
							     (:file "gold-standard" :depends-on ("learning-algorithm"))
							     (:file "hordq" :depends-on ("learning-algorithm"))
							     (:file "rordq" :depends-on ("hordq"))
							     (:file "smdpq" :depends-on ("learning-algorithm"))))
				       ))
		 
		 
		 

 		 
 		 
		 
		 
		 (:module "maxq" :depends-on ("alisp")
			  :in-order-to ((compile-op (load-op "alisp")))
			  :components
			  ((:file "maxq-hierarchy")
			   (:file "task-learner" :depends-on ("maxq-hierarchy"))
			   (:file "var-specs")
			   (:file "parse-trajectory" :depends-on ("maxq-hierarchy"))
			   (:file "goal-language" :depends-on ("maxq-hierarchy"))
			   (:file "greedy-hierarchy-construction" 
				  :depends-on ("maxq-hierarchy" "parse-trajectory" "goal-language" "var-specs"))
			   (:file "relevant-variables"
				  :depends-on ("maxq-hierarchy" "var-specs"))
			   ))

		 
		 (:module "alisp-examples" :depends-on ("env" "envs" "alisp" "misc" "maxq")
			  :in-order-to ((compile-op (load-op "alisp")))
			  :components
			  ((:module "taxi" :components ((:file "td-taxi-prog")
							(:file "td-taxi-prog-features" 
							       :depends-on ("td-taxi-prog"))
							(:file "taxi-maxq")
							(:file "qe-taxi-prog")))))
		 
		 (:module "autoshape" :depends-on ("rl")
			  :components
			  ((:file "autoshape")
			   (:file "pot-fn-learner" :depends-on ("autoshape"))
			   (:file "autodec" :depends-on ("autoshape"))
			   (:file "option")))


		 (:module "test" :depends-on ("envs" "misc")
			  :components ((:file "mdp-test-envs")))))
	
(in-package cl-user)

|#


;; Local variables:
;; mode:lisp
;; outline-regexp:"\\s-*..module"
;; End:
