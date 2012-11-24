(ns core
  (:import [java.io BufferedReader InputStreamReader])

  )


(defn setup
  []
  )

(defn exec
  []
  (let [rdr (BufferedReader. (InputStreamReader. System/in))]
    (doseq [line-in (line-seq rdr)]
      (println :TODO line-in)
      )))

(defn -main [& args]
  (do
    (setup)
    (println "Running exec")
    (exec))
  )
