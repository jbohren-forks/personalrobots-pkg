
(defpackage utils
  (:use common-lisp)
  (:export consec
	   clone
	   undefmethod
	   
	   def-struct-clone-method
	   copy-into
	   same
	   canonicalize
	   def-struct-canonicalize-method
	   gendefstruct
	   copy
	   leave-out
	   ld
	   set-if-unbound
	   as-is
	   
	   get-lambda-list
	   indicator
	   xor
	   nxor
	   below

	   make-hash-table*
	   gethash*
	   has-key
	   hash-table-count*
	   maphash*
	   do-hash
	   gen-hash-table
	   hash-table-test*
	   eql-hash-table
	   hash-table-has-key
	   hash-keys
	   copy-hash-table
	   hash-table-select
	   classify-bin
	   alist-to-hash
	   print-hash-table-readably
	   
	   round-decimal
	   build-symbol-name
	   intern-compound-symbol
	   round-array
	   reshape-array
	   map-array
	   subarray
	   a+
	   mv*
	   a*
	   a/
	   a-
	   inner-product
	   convex-combination
	   array-size-multipliers

	   rand-fn-hist
	   defstub
	   current-time-str
	   check-exact-class
	   results-in-error
	   
	   declare-params
	   reset-params
	   print-params
	   
	   inf-array
	   make-inf-array
	   inf-aref
	   inf-array-bounds
	   inf-array-get-table
	   
	   read-object-from-file
	   sparsify
	   diag
	   transpose
	   array-lp-dist
	   bsearch
	   make-adjustable-array
	   append-to-adjustable-array
	   
	   number-sequence
	   hash-to-alist
	   
	   pprint-float
	   *pprint-num-decimal-places*
	   pprint-matrix
	   pprint-hash
	   with-pprint-dispatch
	   debug-print
	   debug-prompt
	   *debug-level*
	   when-debugging
	   with-debug-indent
	   debug-out
	   set-debug-level
	   reset-debug-level
	   *debug-topics*
	   define-debug-topic

	   wait-until
	   length-exceeds
	   insert-sorted-list
	   insert-at-position
	   designated-list
	   p2alist
	   is-true-list
	   member-equal
	   
	   is-sorted
	   extract-subsequence
	   round-sequence
	   contiguous-blocks
	   mapcans
	   
	   force-format
	   defvars
	   eval-now
	   _f
	   toggle
	   avg
	   avgf
	   adjustf
	   multf
	   divf
	   orf
	   deletef
	   adjoinf
	   set-unless-key-exists
	   maxf
	   minf
	   with-gensyms
	   while
	   until
	   repeat-until 
	   repeat
	   
	   iwhen
	   iunless
	   for-loop
	   
	   condlet
	   aif
	   awhen
	   awhile
	   aand
	   
	   abbrev
	   mvbind
	   dbind
	   dsbind
	   with-struct
	   with-readers

	   mvsetq
	   unbind-slot
	   
	   do-tests
	   do-rand-tests
	   do-boolean-tests
	   tests
	   *rhs*
	   *lhs*
	   
	   
	   do-iterator
	   map-iterator-to-list
	   it
	   
	   define-read-dispatch

	   make-alist-function
	   
	   to-boolean
	   bit-true
	   bit-false

	   map-iterator
	   
	   
	   abs-diff
	   squared-diff
	   between
	   between2

	   default-comparator
	   divisible-by
	   make-evenly-spaced-intervals
	   stail
	   slast
	   sfirst
	   ssecond
	   sthird
	   sfourth
	   sfifth
	   ssixth
	   srest
	   fill-format
	   fill-format-nl
	   with-outfile
	   prompt
	   
	   srs
	   rrs
	   randomized-trials
	   load-relative
	   
	   defaggregator
	   compose
	   nth-arg-fn

	   fn
	   arglist-fn
	   is-standard-equality-test
	   designated-function
	   
	   infty
	   -infty
	   extended-real
	   multiply-zero-by-infinity
	   add-infinity-and-minus-infinity
	   infinite
	   my<
	   my<=
	   my=
	   my>
	   my>=
	   mymax
	   mymin
	   my+
	   my-
	   my*
	   myexp
	   myinv
	   my/
	   
	   argmax
	   maximizing-element
	   minimizing-element
	   argmin
	   
	   check-not-null
	   verify-type
	   
	   change-if-necessary
	   define-memoized-reader
	   compute-memoized
	   
	   symbol<
	   pprint-tabulated-pair
	   bind-pprint-args
	   without-quotes
	   get-mixin-class

	   def-symmetric-method
	   
	   matlab-write
	   matlab-read
	   convert-to-matlab
	   convert-from-matlab
	   
	   
	   split-string
	   
	   vdc-generator
	   n-dimensional-vdc-generator
	   nth-prime
	   
	   
	   mod+
	   mod-
	   mod*
	   mod-inc
	   mod-dec
	   mod-expt
	   aref-mod
	   *2pi*
	   )
	   
  (:export "HELP"
	   "HELP-INT"))


(defpackage "HELP"
  (:use cl)
  (:export 
   "HELP" "HELP-INT"
   ))

 
  

  
	   
(in-package cl-user)

