(ns core
  (:import [java.io BufferedReader InputStreamReader])
  (:use [overtone.core])
  )

(defn reset [a b]
  b)

(defn dubstep-limiter
  [agent-state]
  (try
    (->
     agent-state
     ;; Ensure all fields >= 1
     ((fn [agent-state]
          (reduce #(if (<= (get %1 %2) 0)
                     (assoc %1 %2 1)
                     %1)
                  agent-state (keys agent-state))
        ))
     )
    (catch Throwable ex
      (println "Resetting; Caught in limiter: " ex)
      {:bpm 120 :wobble 1 :note 50 :snare-vol 1 :kick-vol 1 :v 1}))
  )

(defsynth dubstep [bpm 120 wobble 1 note 50 snare-vol 1 kick-vol 1 v 1]
 (let [trig (impulse:kr (/ bpm 120))
       freq (midicps note)
       swr (demand trig 0 (dseq [wobble] INF))
       sweep (lin-exp (lf-tri swr) -1 1 40 3000)
       wob (apply + (saw (* freq [0.99 1.01])))
       wob (lpf wob sweep)
       wob (* 0.8 (normalizer wob))
       wob (+ wob (bpf wob 1500 2))
       wob (+ wob (* 0.2 (g-verb wob 9 0.7 0.7)))

       kickenv (decay (t2a (demand (impulse:kr (/ bpm 30)) 0 (dseq [1 0 0 0 0 0 1 0 1 0 0 1 0 0 0 0] INF))) 0.7)
       kick (* (* kickenv 7) (sin-osc (+ 40 (* kickenv kickenv kickenv 200))))
       kick (clip2 kick 1)

       snare (* 3 (pink-noise [1 1]) (apply + (* (decay (impulse (/ bpm 240) 0.5) [0.4 2]) [1 0.05])))
       snare (+ snare (bpf (* 4 snare) 2000))
       snare (clip2 snare 1)]

   (out 0    (* v (clip2 (+ wob (* kick-vol kick) (* snare-vol snare)) 1)))))

(def dubstep-agent (agent {:bpm 120 :wobble 1 :note 50 :snare-vol 1 :kick-vol 1 :v 1}))
(def dubstep-conn  (atom nil))

(defn setup
  []
  (do
    (boot-server)
    (let [dubstep-params (dubstep)]
      (reset! dubstep-conn dubstep-params)
      (println "Connected to SUPERCOLLIDER with " @dubstep-conn)
      )))

(defn arg-converter
  [arg]
  (or

   (if (re-matches #"[0-9]+" arg)
     (Long/parseLong arg))
   (if-let [builtin-fun (resolve (symbol arg))]
     builtin-fun)
   (if-let [builtin-fun ({"reset" reset} arg)]
     builtin-fun)
   )
  )

(defn dub-update
  "Updater function which is a convenince to alter dubstep in the supercollider"
  [param & args]
  (let [param (keyword param)
        args  (vec (map arg-converter args))]
    (println "... dub-update... " param args)
    (apply send dubstep-agent (comp
                               dubstep-limiter
                               (fn [agent-state param & args] (try
                                                                ;; (println "WTF?" agent-state param args)
                                                                (apply update-in agent-state [param] args)
                                                                (catch Throwable ex
                                                                  (do (println "Caught in agent: " ex) agent-state)))))
           param args)
    (await dubstep-agent)
    (doseq [[k v] @dubstep-agent]
      (ctl (:id @dubstep-conn) k v)
      )
    )
  )

(defn exec
  []
  (let [rdr (BufferedReader. (InputStreamReader. System/in))]
    (doseq [line-in (line-seq rdr)]
      (println " ... debug ... <- " line-in)
      (try
        (apply dub-update (re-seq #"[\w\+\-\*/]+" line-in))
        (catch Throwable ex
          (println "Caught, cant apply dub-update: " ex)))
      )))

(defn -main [& args]
  (do
    (setup)
    (println "Running exec")
    (exec))
  )
