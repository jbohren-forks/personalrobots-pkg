(in-package motion-planning)

(defgeneric get-cspace (cspace-family params)
  (:documentation "get-cspace CSPACE-FAMILY PARAMS
Get the cspace corresponding to the given params"))

