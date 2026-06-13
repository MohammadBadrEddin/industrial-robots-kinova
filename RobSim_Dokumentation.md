# Dokumentation – RobSim.m

Diese Datei erklärt den gesamten Code in `RobSim.m`. Sie ist so aufgebaut, dass du sie zusammen mit `RobSim.m` (und idealerweise `Aufgabe.md` für die Originalaufgabenstellung) bei Claude hochladen kannst, um dir die Zusammenhänge erklären zu lassen.

`RobSim.m` deckt **alle drei Lab-Assignments** ab (Industrial Robotics – Kinova Gen3, alles in MATLAB-Simulation, kein physischer Roboter):

| Abschnitt im Code | Assignment | Thema |
|---|---|---|
| Zeile 7–58 | **Assignment 1** | Frame Manipulation (3D Frames, homogene Transformationen, Punkttransformation) |
| Zeile 64–170 | **Assignment 2** | Forward Kinematics & Manipulability |
| Zeile 172–268 | **Assignment 3** | Pick & Place + Energieverbrauch |
| Zeile 270–316 | Hilfsfunktionen | Werden von allen drei Assignments verwendet |

Beim Ausführen öffnen sich **5 Figures**:
1. Frames F0–F3 (Assignment 1)
2. Roboter-Animation Home→qF1→qF2→qF3 mit Manipulierbarkeits-Pfeil (Assignment 2)
3. Pick & Place Animation, Joint Space (Assignment 3)
4. Cartesian-Space-Pfad F1→F3 (Assignment 3)
5. Leistungs- und Energie-Plots (Assignment 3)

---

## ASSIGNMENT 1 – Frame Manipulation (`RobSim.m:7-58`)

### Ziel
3D-Koordinatensysteme (Frames) definieren, zeichnen, ineinander umrechnen und Punkte zwischen Frames transformieren.

### 2.3 – Frames definieren und zeichnen (`RobSim.m:7-32`)

Für jeden Frame F1, F2, F3 wird eine **homogene Transformationsmatrix (HTM)** `T0i` aufgebaut:

```matlab
R1  = rotZd(90) * rotYd(-175) * rotXd(-89);
O1  = [-0.78; 0.15; 0.21];
T01 = [R1, O1; 0 0 0 1];
```

- `rotXd`, `rotYd`, `rotZd` (Hilfsfunktionen, `RobSim.m:305-315`) erzeugen die elementaren 3×3-Rotationsmatrizen für Drehungen um die x-, y-, z-Achse (Eingabe in **Grad**, daher `cosd`/`sind`).
- Die Reihenfolge `Rz * Ry * Rx` entspricht der Aufgabenstellung: "Rotation um -89° (x), -175° (y), 90° (z) um die **festen** Achsen von F0" → **extrinsische Rotation**, daher Multiplikation von links nach rechts in der Reihenfolge z-y-x.
- `R1` ist die 3×3-Rotationsmatrix, `O1` der Ursprung von F1 in F0-Koordinaten. Zusammen ergeben sie die 4×4-HTM `T01 = [R1 O1; 0 0 0 1]`.
- `T02`, `T03` analog mit den Werten aus der Aufgabenstellung (Abschnitt 2.3).

**`drawFrame(O, R)`** (`RobSim.m:296-303`):
```matlab
function drawFrame(O, R)
    scale = 0.1;
    colors = {'r', 'g', 'b'};
    for i = 1:3
        ep = O + scale * R(:,i);
        plot3([O(1) ep(1)], [O(2) ep(2)], [O(3) ep(3)], colors{i}, 'LineWidth', 2);
    end
end
```
- `R(:,i)` ist die i-te Spalte von R = die Richtung der x-, y-, bzw. z-Achse des Frames, ausgedrückt in F0-Koordinaten.
- Für jede Achse wird eine Linie vom Ursprung `O` bis `O + scale*Achse` gezeichnet (rot=x, grün=y, blau=z – Standardkonvention).
- F0 selbst wird mit `drawFrame([0;0;0], eye(3))` gezeichnet (Identität = Weltframe).

### 2.4 – Transformationen zwischen Frames (`RobSim.m:34-43`)

```matlab
T21 = inv(T02) * T01;   % Pose von F1 ausgedrueckt in F2
T32 = inv(T03) * T02;   % Pose von F2 ausgedrueckt in F3
T20 = inv(T02);         % Pose von F0 ausgedrueckt in F2
```

**Kernkonzept:** `T0i` beschreibt "wo ist Fi, gesehen von F0". Um die Pose von Fi aus Sicht von Fj zu bekommen, rechnet man:
```
Tji = inv(T0j) * T0i
```
- `inv(T02)` "macht die Transformation nach F2 rückwärts", angewendet auf `T01` (F1 aus Sicht von F0) ergibt `T21` = F1 aus Sicht von F2.
- `T20 = inv(T02)` ist der Spezialfall: F0 aus Sicht von F2 = genau die Inverse von "F2 aus Sicht von F0".

### 2.5 – Punktkoordinaten transformieren (`RobSim.m:45-58`)

```matlab
P3 = [1; 0; 0];    % Punkt P3 gegeben in F3
P2 = [-1; 0; -1];  % Punkt P2 gegeben in F2

P3_F0 = T03          * [P3; 1]; P3_F0 = P3_F0(1:3);
P3_F1 = inv(T01)*T03 * [P3; 1]; P3_F1 = P3_F1(1:3);
```

- `[P3; 1]` erweitert den 3D-Punkt zu **homogenen Koordinaten** (4×1), damit man ihn mit einer 4×4-HTM multiplizieren kann.
- `T03 * [P3;1]` rechnet den Punkt von "F3-Koordinaten" nach "F0-Koordinaten" um (Standard-HTM-Anwendung: `p0 = T0i * pi`).
- `inv(T01) * T03 * [P3;1]`: erst nach F0 (`T03 * P3`), dann von F0 nach F1 (`inv(T01)`) – also Verkettung von zwei Transformationen.
- `(1:3)` schneidet die homogene 1 wieder ab, übrig bleibt der 3D-Punkt.

---

## ASSIGNMENT 2 – Forward Kinematics & Manipulability (`RobSim.m:64-170`)

### Ziel
Eigene Vorwärtskinematik (FK) des Kinova Gen3 (7-DoF) über DH-Parameter implementieren, gegen MATLABs Robotermodell validieren, und die **Manipulierbarkeit** (Jacobian-basiert) analysieren.

### Robotermodell laden (`RobSim.m:65-67`)
```matlab
gen3 = loadrobot("kinovaGen3");
gen3.DataFormat = 'column';
eeName = 'EndEffector_Link';
```
- `loadrobot` lädt das offizielle URDF-Modell des Kinova Gen3 aus der Robotics System Toolbox – das ist unser "digitaler Zwilling" / Referenz für alle Vergleiche.
- `DataFormat = 'column'`: Gelenkwinkel werden als Spaltenvektor (7×1) übergeben.

### DH-Parameter-Tabelle (`RobSim.m:69-82`)
```matlab
DH = [
    pi,      0,                                0;   % Basis
    pi/2,   -(0.1564 + 0.1284),                0;   % Gelenk 1
    ...
];
```
Jede Zeile = `[alpha, d, theta_offset]` für ein DH-Frame (a=0 für alle Glieder, klassisches DH).
- `alpha`: Verdrehung der Gelenkachse (meist ±π/2)
- `d`: Verschiebung entlang der z-Achse – ist hier als **Summe zweier physikalischer Segmentlängen** aus dem URDF angegeben (zwei Bodies pro "logischem" Gelenk)
- `theta_offset`: konstanter Offset zum Gelenkwinkel `q_i` (kommt daher, dass die DH-Konvention und die URDF-Nullstellung nicht exakt übereinstimmen)

**`dhFrame(theta, d, a, alpha)`** (`RobSim.m:284-292`): baut die **Standard-DH-Transformationsmatrix**
```
T = Rot_z(theta) * Trans_z(d) * Trans_x(a) * Rot_x(alpha)
```
als 4×4-HTM. Das ist die Grundformel, mit der man von Frame i-1 zu Frame i kommt.

**`forwardKin(q, DH)`** (`RobSim.m:272-282`):
```matlab
function T = forwardKin(q, DH)
    T = dhFrame(DH(1,3), DH(1,2), 0, DH(1,1));   % Basis-Frame
    for i = 1:7
        theta = q(i) + DH(i+1, 3);
        T = T * dhFrame(theta, DH(i+1,2), 0, DH(i+1,1));
    end
    T = T * [eye(3), [0; 0; 0.12]; 0 0 0 1];     % Tool-Offset
end
```
- Startet bei der Basis und **verkettet** (Matrixmultiplikation) die Transformationen aller 7 Gelenke: `T08 = T01 * T12 * T23 * ... * T78`.
- `theta = q(i) + DH(i+1,3)`: der tatsächliche DH-Winkel ist der Gelenkwinkel `q(i)` **plus** den Offset aus der Tabelle.
- Am Ende wird der **Tool-Offset** `[0,0,0.12]` (12 cm in z-Richtung des Interface-Frames) angehängt – das entspricht Aufgabe 3.2.4.
- Das Ergebnis `T` ist `T08(q)` – die gesamte Pose des Endeffektors in Weltkoordinaten, abhängig vom Gelenkwinkelvektor `q`.

### Konfigurationen (`RobSim.m:84-91`)
```matlab
q_home = [0; 15; 180; -130; 0; 55; 90] * pi/180;
qF1    = [16.39; 299.74; ...] * pi/180;
```
- Vorgegebene Gelenkwinkel-Sätze (Grad → Bogenmaß). `qF1/qF2/qF3` sind so gewählt, dass der Endeffektor genau in den Frames **F1/F2/F3 aus Assignment 1** landet (Verbindung zwischen den Assignments!).

### Figure-Setup mit expliziten Axes (`RobSim.m:93-103`)
```matlab
fig2 = figure;
ax2  = axes(fig2);
show(gen3, q_home, 'PreservePlot', false, 'Parent', ax2);
```
- `axes(fig2)` erzeugt ein explizites Axes-Handle `ax2`. Alle weiteren Zeichenbefehle (`show`, `quiver3`) bekommen `'Parent', ax2` bzw. `ax2` als erstes Argument – das verhindert, dass MATLAB versehentlich in die falsche Figure zeichnet (z.B. in die Frame-Figure von Assignment 1).

### vm an der Home-Konfiguration (`RobSim.m:105-111`)
```matlab
J_home     = geometricJacobian(gen3, q_home, eeName);
[V_h, D_h] = eig(J_home(1:3,:) * J_home(1:3,:)');
[~, id_h]  = max(diag(D_h));
vm_home    = V_h(:, id_h);
```
- `geometricJacobian` liefert den **6×7 Jacobian** `J` (erste 3 Zeilen = translatorisch, letzte 3 = rotatorisch). `J(1:3,:)` ist `J_t`.
- **Manipulierbarkeitsellipse:** Wenn man alle Gelenkgeschwindigkeiten `q̇` mit `‖q̇‖=1` durch `J_t` schickt, ergibt sich im kartesischen Raum eine **Ellipse**. Ihre Hauptachsen sind die Eigenvektoren von `J_t·J_tᵀ`, ihre Halbachsenlängen die Wurzeln der Eigenwerte.
- `eig(J_t*J_t')` gibt Eigenvektoren `V` (Spalten) und Eigenwerte `D` (Diagonalmatrix).
- `vm = V(:,id)` mit `id` = Index des **größten** Eigenwerts → Richtung, in der der Endeffektor mit der gegebenen Konfiguration **am leichtesten/schnellsten** bewegt werden kann (= größte Halbachse der Ellipse).

### Animationsschleife (`RobSim.m:113-170`)

```matlab
for k = 1:3
    q_start = configs{k};
    q_end   = configs{k+1};
    dq      = wrapToPi(q_end - q_start);
    for s = 1:steps
        alpha_t = s / steps;
        q       = q_start + alpha_t * dq;
        ...
```

- Iteriert über die 3 Segmente Home→qF1, qF1→qF2, qF2→qF3.
- **`wrapToPi(q_end - q_start)`**: Gelenkwinkel sind periodisch (360°-Wraparound). Eine naive lineare Interpolation `q = (1-α)*q_start + α*q_end` könnte den **langen Weg** um 360° herum nehmen (z.B. von 15° auf 299.74° statt -60.26°, was physikalisch dieselbe Pose ist). `wrapToPi` rechnet die Winkeldifferenz auf den Bereich `[-π,π]` um → **kürzester Weg pro Gelenk**, keine unnötigen Volldrehungen / Selbstkollisionen.
- Pro Zwischenschritt:
  - `T08 = forwardKin(q, DH)` → eigene FK, `p = T08(1:3,4)` = aktuelle Endeffektorposition
  - Jacobian + Eigenzerlegung wie oben → `vm` für die aktuelle Pose
  - `show(...)` zeichnet den Roboter in der aktuellen Konfiguration
  - `quiver3(...)` zeichnet `vm` (skaliert mit 0.1) als **magenta Pfeil** am Endeffektor; der alte Pfeil wird vorher gelöscht (`delete(h_vm)`), damit nur der aktuelle sichtbar ist

### Vergleich eigene FK vs. MATLAB (`RobSim.m:151-169`)
```matlab
T08      = forwardKin(q_end, DH);
T_matlab = getTransform(gen3, q_end, eeName);
dp       = norm(T08(1:3,4) - T_matlab(1:3,4));
R_diff   = T08(1:3,1:3)' * T_matlab(1:3,1:3);
dtheta   = acos(min(1, (trace(R_diff) - 1) / 2)) * 180/pi;
```
- `getTransform` ist MATLABs **eingebaute** FK (Referenz/"ground truth").
- `dp`: euklidischer Abstand der beiden Positionsvektoren → Validierung der eigenen DH-Implementierung (sollte ≈ 0 sein).
- `R_diff = R_eigen' * R_matlab`: die "Differenz-Rotation" zwischen beiden Rotationsmatrizen. Für `R_diff = I` (identisch) gilt `trace(R_diff) = 3`.
- `dtheta = acos((trace(R_diff)-1)/2)`: Standardformel, um aus einer Rotationsmatrix den **Rotationswinkel** der Achse-Winkel-Darstellung zu extrahieren → Orientierungsfehler in Grad.
- Am Ende wird zusätzlich `vm` an der Zielkonfiguration (`vm_end`) berechnet und ausgegeben – zusammen mit `vm_home` ergibt das die Datenbasis für die **Interpretation des Manipulierbarkeits-Verhaltens** (Start- vs. Endkonfiguration), die schriftlich im Bericht erfolgt.

---

## ASSIGNMENT 3 – Pick & Place + Energieverbrauch (`RobSim.m:172-268`)

### Ziel
Kollisionsfreie Bewegungsplanung (Joint Space **und** Cartesian Space) zu den Frames F1/F2/F3, plus Berechnung des Energieverbrauchs aus simulierten Dynamikdaten.

### Aufgabenkontext (aus Aufgabenblatt 4.2)
- Zwei Flaschen B1 (bei F1) und B2 (bei F3).
- Ziel: B2 → F2, B1 → F3.
- Reihenfolge: F3 muss erst **frei** werden (B2 weg), bevor B1 dort abgelegt werden kann.

### 4.3a – Joint Space Trajektorie (`RobSim.m:176-209`)

**Wegpunkt-Sequenz:**
```matlab
pp_configs = {q_home, qF3, qF2, qF1, qF3, q_home};
% Home -> F3 (B2 greifen) -> F2 (B2 ablegen) -> F1 (B1 greifen) -> F3 (B1 ablegen) -> Home
```

**"Unwrapping" der Wegpunkte (`RobSim.m:182-187`):**
```matlab
wayPoints(:,1) = pp_configs{1};
for i = 2:n_wp
    dq = wrapToPi(pp_configs{i} - pp_configs{i-1});
    wayPoints(:,i) = wayPoints(:,i-1) + dq;
end
```
- Gleiches Prinzip wie bei Assignment 2, aber jetzt für eine **ganze Kette** von Wegpunkten: jede Differenz wird auf den kürzesten Weg gewrappt und **aufsummiert**, sodass `wayPoints` eine durchgehende (nicht mehr 360°-periodische) Folge von Gelenkwinkeln ist – Voraussetzung für eine glatte Trajektorie ohne Sprünge.

**`trapveltraj`** (`RobSim.m:193-194`):
```matlab
[q_traj, qd_traj, qdd_traj, t_traj] = trapveltraj(wayPoints, numSamples, 'EndTime', seg_time);
```
- Erzeugt für jedes Gelenk ein **Trapez-Geschwindigkeitsprofil** (Beschleunigen → konstante Geschwindigkeit → Abbremsen) zwischen den Wegpunkten.
- Output: `q_traj` (Position), `qd_traj` (Geschwindigkeit `q̇`), `qdd_traj` (Beschleunigung `q̈`), jeweils 7×numSamples, und `t_traj` (Zeitvektor).
- `EndTime = seg_time` (2 s): jedes der 5 Segmente dauert 2 Sekunden.
- Diese drei Größen (`q, q̇, q̈`) sind genau das, was später `inverseDynamics` braucht.

**Animation** (`RobSim.m:196-209`): wie bei Assignment 2, aber mit eigener Figure (`fig3`/`ax3`) und über alle `numSamples` Zeitschritte der Trajektorie.

### 4.3b – Cartesian Space Trajektorie (`RobSim.m:211-240`)

Demonstriert exemplarisch das Segment **F1 → F3** als **kontinuierlichen Pfad im Arbeitsraum** (statt im Gelenkraum).

```matlab
T_F1 = getTransform(gen3, qF1, eeName);
T_F3 = getTransform(gen3, qF3, eeName);

T_cart = transformtraj(T_F1, T_F3, [0 seg_time], tSamples_cart);
```
- `transformtraj` interpoliert **zwischen zwei HTMs**: die Position linear, die Orientierung über **SLERP** (Spherical Linear Interpolation der Rotation) – das Ergebnis ist eine glatte gerade Linie im kartesischen Raum, nicht im Gelenkraum.
- Output `T_cart` ist ein 4×4×numCart-Array: eine HTM pro Zeitschritt.

**Inverse Kinematik pro Zeitschritt** (`RobSim.m:219-226`):
```matlab
ik      = inverseKinematics('RigidBodyTree', gen3);
q_init  = wrapToPi(qF1);
for k = 1:numCart
    q_cart(:,k) = ik(eeName, T_cart(:,:,k), weights, q_init);
    q_init      = q_cart(:,k);   % "warm start" fuer naechsten Schritt
end
```
- Da der Roboter im Gelenkraum gesteuert wird, muss für **jede** kartesische Zwischenpose `T_cart(:,:,k)` die passende Gelenkkonfiguration per IK gelöst werden.
- `q_init = wrapToPi(qF1)`: Startschätzung für die IK – muss innerhalb der Gelenkgrenzen liegen (Kinova-Gelenke 2/4/6 sind limitiert; `qF1` selbst enthält Werte >180°, die zwar für FK kein Problem sind, aber von `inverseKinematics` als Verletzung der Limits abgelehnt werden – `wrapToPi` gibt die physikalisch identische, aber gültige Darstellung).
- **Warm Start**: die Lösung von Schritt `k` wird als Startwert für Schritt `k+1` verwendet → IK konvergiert schnell und liefert eine **stetige** Gelenktrajektorie (keine Sprünge zwischen ähnlichen kartesischen Posen).

**Animation** in eigener Figure (`fig4`/`ax4`).

### 4.4 – Energieverbrauch (`RobSim.m:242-268`)

```matlab
gen3.Gravity = [0 0 -9.81];

tau = zeros(7, numSamples);
for k = 1:numSamples
    tau(:,k) = inverseDynamics(gen3, q_traj(:,k), qd_traj(:,k), qdd_traj(:,k));
end

P = sum(tau .* qd_traj, 1);
E = cumtrapz(t_traj, P);
```

- `gen3.Gravity = [0 0 -9.81]`: setzt die Erdbeschleunigung im Modell (Default ist `[0 0 0]`) – ohne das würden die berechneten Momente die Gewichtskraft der Armsegmente nicht berücksichtigen.
- **`inverseDynamics(gen3, q, q̇, q̈)`**: berechnet aus Position, Geschwindigkeit und Beschleunigung aller Gelenke die dafür nötigen **Gelenkmomente** `τ` (Newton-Euler- bzw. Lagrange-Dynamik) – inklusive Gravitations-, Coriolis- und Trägheitsanteilen.
- **Leistung:** `P(t) = Σᵢ τᵢ(t) · q̇ᵢ(t)` – mechanische Leistung = Kraft/Moment × Geschwindigkeit, summiert über alle 7 Gelenke (`tau .* qd_traj` ist elementweise, `sum(...,1)` summiert über die Gelenke/Zeilen).
- **Energie:** `E(t) = ∫₀ᵗ P(t') dt'` – `cumtrapz` integriert numerisch (Trapezregel) **kumulativ**, sodass `E(t)` die bis zum Zeitpunkt `t` verbrauchte Energie ist. `E(end)` ist die Gesamtenergie der ganzen Pick & Place-Sequenz.

**Plots** (`RobSim.m:253-268`):
- Subplot 1: `P(t)` – Leistungskurve
- Subplot 2: `E(t)` – kumulierte Energie
- `xline(i*seg_time, '--', pp_labels{i+1})`: vertikale, beschriftete Linien an den Segmentgrenzen (alle 2 s) → zeigt, **welche Bewegungsphase** (z.B. "F1: B1 greifen") wie viel zur Leistung/Energie beiträgt – Grundlage für die Interpretation laut Aufgabe 4.4 ("Interpret the variation of energy consumption in different motion phases").

---

## Querverbindungen zwischen den Assignments

- **F1/F2/F3 (Assignment 1)** ↔ **qF1/qF2/qF3 (Assignment 2)**: die Gelenkwinkel sind so gewählt, dass `T08(qFi)` ≈ `T0i` (Endeffektor erreicht genau die in Assignment 1 definierten Frames).
- **wrapToPi** taucht zweimal auf: einmal für eine einzelne Interpolation (Assignment 2, `RobSim.m:120`) und einmal kumulativ für eine ganze Wegpunktkette (Assignment 3, `RobSim.m:182-187`) – gleiches Grundprinzip, unterschiedlicher Maßstab.
- **`trapveltraj`/`q_traj, qd_traj, qdd_traj`** (Assignment 3a) sind die Eingabe für `inverseDynamics` (Assignment 3 Energie) – ohne die Trajektorienplanung mit definierten Geschwindigkeiten/Beschleunigungen gäbe es keine sinnvollen `τ(t)`.
