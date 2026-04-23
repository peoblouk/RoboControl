; ==========================================
; node1_slot1.gcode
; node1 = right robot
; sequence: HOME -> pick from C -> place to B -> HOME
;
; ASSUMPTIONS:
; - wcofs 0 0 0
; - HOME approx:
;   X108 Y0 Z116 P-53
; - C tuned pick point:
;   X225 Y0 Z13 P-75
; - B tuned place point:
;   X63 Y-230 Z15 P-65
; ==========================================

G21
G90
F700

M10
G4 P120

; HOME
G1 X108 Y0 Z116 P-53 F3000
G4 P60

; --------------------------
; HOME -> C
; --------------------------
G1 X170 Y0 Z80 P-75 F3500
G1 X200 Y0 Z50 P-75 F2500
G1 X218 Y0 Z28 P-75 F1800
G1 X223 Y0 Z20 P-75 F900
G1 X225 Y0 Z13 P-75 F180
G4 P220

; grab
M11
G4 P450

; --------------------------
; C -> safe retreat
; keep same pitch until safely out
; --------------------------
G1 X223 Y0 Z20 P-75 F220
G1 X218 Y0 Z28 P-75 F900
G1 X200 Y0 Z50 P-75 F1800
G1 X170 Y0 Z80 P-75 F2500
G4 P60

; --------------------------
; move toward B faster
; --------------------------
G1 X110 Y-150 Z60 P-65 F3500
G1 X63 Y-230 Z35 P-65 F2500
G4 P60

; place at B slowly
G1 X63 Y-230 Z15 P-65 F220
G4 P220

; release
M10
G4 P220

; lift from B
G1 X63 Y-230 Z35 P-65 F900
G4 P60

; return HOME
G1 X110 Y-120 Z90 P-45 F3500
G1 X108 Y0 Z116 P-53 F3500
G4 P60

M30
