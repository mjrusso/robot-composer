(ns core
  )


(defn setup
  []
  )

(defn exec
  []
  (doseq [_slug_ (repeat true)]
    (let [input     (read-line)]
      (println :TODO input)
      )
    ))

(defn restart
  []
  (do
    (setup)
    (exec)))
