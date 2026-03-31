; ===============================
; PICK & PLACE TEMPLATE
; ===============================
; Before first real run:
; 1) tune WORK <-> BASE offsets
; 2) start with higher Z_SAFE and lower speeds
; 3) verify M10/M11 direction (open/close)
;
; Coordinates are in WORK frame [mm]
; P is tool pitch [deg]

G21                 ; mm
G90                 ; absolute mode
F800                ; default feed [mm/min]

; -------- USER TUNE POINTS --------
; PICK:  X=40  Y=40  Z=12
; PLACE: X=-40 Y=60  Z=12
; SAFE Z for travel: Z=70
; APPROACH Z: Z=25

; Start safe + gripper open
M10
G4 P250
G0 X0 Y0 Z70 P20

; ---------- PICK ----------
G0 X40 Y40 Z70
G1 X40 Y40 Z25 F700
G1 X40 Y40 Z12 F300
M11
G4 P300
G1 X40 Y40 Z25 F450
G1 X40 Y40 Z70 F700

; ---------- PLACE ----------
G0 X-40 Y60 Z70
G1 X-40 Y60 Z25 F700
G1 X-40 Y60 Z12 F300
M10
G4 P250
G1 X-40 Y60 Z25 F450
G1 X-40 Y60 Z70 F700

; End
G0 X0 Y0 Z70
M30
