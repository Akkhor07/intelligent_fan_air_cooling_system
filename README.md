# Intelligent Fan Air Cooling System

Design an intelligent fan cooling system to moderate temperatures in a building to eliminate or reduce the need for air conditioning systems, using fuzzy-logic-controlled night-time ventilation.

> \*\*Challenge brief:\*\* MATLAB \& Simulink Sustainability \& Renewable Energy Challenge — \*Intelligent Fan Air Cooling System\*. Impact: contribute to energy and carbon footprint reduction. Expertise: sustainability \& renewable energy, control, modeling \& simulation, optimization.

\---

## What's in this repo

|File|Description|
|-|-|
|`myflcc.fis`|Mamdani fuzzy inference system — 2 inputs (Indoor Temp, Outdoor Temp), 1 output (Fan Speed), 9 rules|
|`IntelligentFan.slx`|Original BLDC motor drive model: Fuzzy Logic Controller → PI speed loop → VSI → commutation logic → electromechanical BLDC dynamics|
|`building\_cooling\_extension.m`|**New.** ODE-based building thermal (RC) model + 3-way controller comparison (Fuzzy / PID / Bang-bang), Pareto trade-off analysis, long-horizon noisy-weather robustness testing|
|`closed\_loop\_manual\_build\_guide.md`|**New.** Step-by-step manual build guide (block names, library paths, parameters) for wiring `myflcc.fis` into a real closed-loop Simulink thermal model|
|`IntelligentFan\_ClosedLoop.slx`|**New.** Closed-loop Simulink model built from the guide above — couples the actual Fuzzy Logic Controller block to a Building Thermal Plant subsystem and a reduced-order motor lag, validated over a 240-hour (10-day) run|
|`figure1.png`, `Figure2.png`, `figure3.png`|**New.** Result plots from `building\_cooling\_extension.m` — indoor temperature comparison, fan speed comparison, and the energy/comfort Pareto trade-off|
|`Simulink Figure.png`|**New.** Scope capture from the closed-loop Simulink model's 10-day validation run|
|`LICENSE`|MIT License|

\---

## Background: what the original model did (and didn't) show

The original `IntelligentFan.slx` is a **motor-drive feasibility model**, not a building simulation. Its indoor/outdoor temperatures are fixed constants (25 °C / 12 °C), so it only demonstrates that the fuzzy controller's single speed command can actually be achieved by the BLDC drive hardware (PI loop, inverter, commutation, motor dynamics) — it says nothing about whether the strategy actually moderates a building's temperature over a day/night cycle.

This repo's additions close that gap: an actual thermal plant, a realistic diurnal weather profile, and a genuine closed feedback loop, first prototyped quickly in MATLAB (`building\_cooling\_extension.m`) and then rebuilt natively in Simulink (`IntelligentFan\_ClosedLoop.slx`) using the real `myflcc.fis` block.

\---

## Quick start

**Requirements:** MATLAB R2020a+ (tested on R2024b), Fuzzy Logic Toolbox. Control System Toolbox optional (only needed if you re-derive PID gains with `pidtune` rather than using the values already in the script).

### Run the MATLAB analysis

```matlab
building\_cooling\_extension            % runs 3-controller comparison + plots
```

Set `USE\_FIS\_FILE = false` at the top if Fuzzy Logic Toolbox isn't available — it falls back to a hand-coded equivalent rule table.

### Build/run the closed-loop Simulink model

Follow `closed\_loop\_manual\_build\_guide.md` to build it block-by-block, or open `IntelligentFan\_ClosedLoop.slx` if already built. Recommended solver: `ode15s` (the model mixes a fast motor lag with a slow thermal plant — a genuinely stiff system). Test at `StopTime = 48` hours first before running the full 240-hour (10-day) simulation.

\---

## Results

### Controller comparison (48-hour clean sine weather profile)

!\[Indoor temperature under three fan-control strategies](figure1.png)

|Controller|Energy\*|Comfort %|
|-|-|-|
|Fuzzy|13.40|88.1|
|PID (properly tuned)|12.17|74.0|
|Bang-bang|25.89|79.0|

\*Energy in normalized `(speed/speedMax)^3`-hours, a fan-affinity-law proxy for power draw.

!\[Commanded fan speed for each controller](figure2.png)

### Energy vs. comfort trade-off (PID gain sweep)

!\[Energy vs comfort Pareto trade-off](figure3.png)

Sweeping PID aggressiveness (`Kp = 2..25`) traces out a full energy/comfort Pareto curve. The Fuzzy controller sits **above** this curve — for the same energy budget, no PID gain setting matches Fuzzy's comfort level, and Fuzzy is Pareto-superior to Bang-bang outright. This is the strongest evidence in the project that the fuzzy strategy is a genuinely better design choice, not just a differently-tuned one.

### Closed-loop Simulink validation

!\[Simulink closed-loop Tin vs Tout, 10-day run](simulink\_figure.png)

The rebuilt Simulink model, run over 10 simulated days with `ode15s`, shows `Tin` settling into a stable, repeating \~23.5–26.5 °C band despite `Tout` swinging the full \~14–30 °C range — consistent with the MATLAB-script result, providing cross-validation between the two independent implementations.

\---

## Known issue found during validation

Inspecting the fan-speed trace from the closed-loop simulation revealed the controller barely uses its full 16–30 speed range, even during the "free night cooling" scenario (indoor hot, outdoor cold) that the whole project is built around. Tracing this back to `myflcc.fis`:

```
\[Rules]
...
3 1, 2 (1) : 1     % Indoor=Hot, Outdoor=Cold -> Speed=Normal   (should be Fast)
```

**Recommended fix:** change this rule's output from `2` (Normal) to `3` (Fast) in `myflcc.fis`, then re-validate — this should let the controller fully exploit the free night-cooling opportunity it currently under-uses.

\---

\---

## License

This project is licensed under the MIT License — see [`LICENSE`](LICENSE) for details. Note this covers the code and models in this repository; the referenced MgO-PVDF paint research (Das, Rudra, Maurya \& Saha, 2023) remains under its original publisher's copyright and is cited here for reference only, not redistributed.

## Future work

* Fix the `myflcc.fis` rule-table issue above and re-run both the MATLAB and Simulink validations.
* Tune the FIS membership functions against the energy/comfort metrics using ANFIS instead of hand-picked triangles.
* Model the reflective paint's effect as an explicit solar-gain reduction term, to quantify its contribution separately from fan ventilation.
* Extend the thermal plant to a multi-zone model, and/or validate against real historical weather data instead of a synthetic diurnal profile.
* Integrate smart-grid / IoT connectivity and occupancy-based speed control, as outlined in the original project vision.

