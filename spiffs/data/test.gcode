; ==========================================
; Simulace "pick & place" bez gripper cmd
; ==========================================
G21
G90

; Bezpecna pozice
G1 X220 Y0 Z140 F1000
G4 P300

; Nad "pickup"
G1 X200 Y0 Z120
G4 P300

; Sestup kousek nad zem (uprav si Z podle reality)
G1 X200 Y0 Z80
G4 P800    ; simulace uchopu

; Zvednout
G1 X200 Y0 Z120
G4 P200

; Presun nad "place"
G1 X240 Y0 Z120
G4 P300

; Sestup
G1 X240 Y0 Z80
G4 P800    ; simulace pusteni

; Zvednout a konec
G1 X240 Y0 Z120
G1 X220 Y0 Z140
M30