//! Lumped thermal masses: the state that makes a cold engine sound cold.
//!
//! Everything acoustic in this simulator hangs off a temperature. The speed of
//! sound in a pipe is `sqrt(gamma R T)`, so every resonance the exhaust has is
//! proportional to `sqrt(T)`; the exhaust gas temperature itself is set by how
//! much heat the combustion chamber walls took out of the charge; and the
//! mechanical noise floor is set by friction, which is set by how thick the oil
//! is. All three of those were constants. This module makes them state.
//!
//! The model is the crudest one that is still a model: a lumped capacity per
//! body, heated by the flux the solver already computes and cooled by a
//! conductance to a sink.
//!
//! ```text
//! C dT/dt = Q_in - h (T - T_sink)
//! ```
//!
//! A lumped capacity is only honest while the body is small compared with the
//! distance heat diffuses through it in a time constant — which a 1.5 mm pipe
//! wall comfortably is, and a whole engine block is not. See
//! [`WARM_UP_MASS_FRACTION`] for what is done about that.

use std::f64::consts::PI;

use crate::physics::plumbing::{ExhaustSystem, PipeSection};

// ---------------------------------------------------------------------------
// Material and gas constants
// ---------------------------------------------------------------------------

/// Specific heat capacity of the engine's structural metal [J/(kg K)].
///
/// Between cast iron (450) and aluminium (900); a dressed engine is a mixture
/// of both plus its steel fasteners, and the coolant and oil it carries are
/// lumped in with it.
pub const METAL_SPECIFIC_HEAT: f64 = 520.0;

/// Fraction of the dressed block mass that follows the combustion chamber [-].
///
/// The whole casting does not warm up together. With the thermostat shut the
/// radiator and its coolant are isolated, and the far end of the sump is
/// hundreds of seconds of conduction away from the bores, so what actually
/// responds to the first two minutes of running is the metal *around the
/// chambers* and the coolant in the block. Measured warm-ups imply an effective
/// capacity near a quarter of the dressed mass; using the whole casting puts a
/// road engine twenty minutes from operating temperature, which is wrong by a
/// factor of five.
pub const WARM_UP_MASS_FRACTION: f64 = 0.25;

/// Density of the steel exhaust tubing [kg/m^3].
pub const PIPE_DENSITY: f64 = 7_800.0;
/// Specific heat capacity of the steel exhaust tubing [J/(kg K)].
pub const PIPE_SPECIFIC_HEAT: f64 = 490.0;
/// Wall thickness of the exhaust tubing [m].
///
/// 1.5 mm is what mandrel-bent 16-gauge primaries and a production tailpipe are
/// both near enough to; it is the single number that sets how long an exhaust
/// takes to come up to temperature.
pub const PIPE_WALL_THICKNESS: f64 = 1.5e-3;

/// Thermal conductivity of exhaust gas at working temperature [W/(m K)].
pub const EXHAUST_CONDUCTIVITY: f64 = 0.065;
/// Dynamic viscosity of exhaust gas at working temperature [Pa s].
pub const EXHAUST_VISCOSITY: f64 = 3.8e-5;
/// Prandtl number of exhaust gas [-].
pub const EXHAUST_PRANDTL: f64 = 0.72;
/// Specific heat at constant pressure of exhaust gas [J/(kg K)].
pub const EXHAUST_CP: f64 = 1_150.0;

/// Enhancement of the gas-side film coefficient by flow pulsation [-].
///
/// Dittus-Boelter describes steady pipe flow. Exhaust flow is not steady: it
/// arrives as a train of blowdown slugs that scour the boundary layer flat on
/// every event, and manifold heat-transfer measurements come back around twice
/// the steady-flow correlation because of it.
pub const PULSATION_ENHANCEMENT: f64 = 2.0;

/// Free-convection and radiation coefficient on the outside of a pipe [W/(m^2 K)].
///
/// A hot pipe in still air under a car: natural convection is worth perhaps
/// 10 W/(m^2 K), and at 900 K the radiative part is worth rather more than
/// that, so the two are carried together as one linearised coefficient.
pub const PIPE_EXTERNAL_COEFFICIENT: f64 = 30.0;

/// Conductance from the combustion chamber surface to the bulk block [W/(m^2 K)].
///
/// The chamber wall does not sit at the block's bulk temperature: the heat
/// Woschni takes out of the charge has to cross the head metal, the coolant
/// film and whatever deposit is on the inside before it reaches the coolant, and
/// that drop is what makes a fired chamber wall run near 450 K against a 363 K
/// coolant. Calibrated so a warm engine at idle lands on the 450 K this model
/// used to assume outright.
pub const HEAD_CONDUCTANCE: f64 = 1_200.0;

/// Highest chamber wall temperature the head is allowed to reach [K].
///
/// A linear conductance keeps raising the wall as the flux grows, but an
/// aluminium head does not: past this the coolant is boiling nucleate at the
/// hot spots, which pins the surface, and past it by much the casting is
/// failing rather than running. The ceiling is the material's, not a tuning
/// knob.
pub const MAX_WALL_TEMPERATURE: f64 = 600.0;

// ---------------------------------------------------------------------------
// A lumped body
// ---------------------------------------------------------------------------

/// One lumped thermal mass with a convective path to a sink.
///
/// ```text
/// C dT/dt = Q_in - h (T - T_sink)
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ThermalMass {
    /// Bulk temperature of the body [K].
    pub temperature: f64,
    /// Heat capacity `C` [J/K].
    pub capacity: f64,
    /// Conductance `h` to the sink [W/K].
    pub conductance: f64,
    /// Temperature of the sink the body loses to [K].
    pub sink_temperature: f64,
}

impl ThermalMass {
    /// A body at a temperature, with a capacity and a path to a sink.
    pub fn new(temperature: f64, capacity: f64, conductance: f64, sink_temperature: f64) -> Self {
        Self {
            temperature,
            capacity: capacity.max(1e-3),
            conductance: conductance.max(1e-9),
            sink_temperature,
        }
    }

    /// First-order time constant `tau = C / h` [s].
    pub fn time_constant(&self) -> f64 {
        self.capacity / self.conductance
    }

    /// Temperature the body settles at under a steady input [K].
    pub fn equilibrium(&self, heat_in: f64) -> f64 {
        self.sink_temperature + heat_in / self.conductance
    }

    /// Advances the body by `dt` under a steady heat input [W].
    ///
    /// Integrated in closed form rather than by an Euler step. The frame here is
    /// milliseconds and the pipe time constants are tens of seconds, so an
    /// explicit step would be perfectly stable — but the exponential is the
    /// exact solution of the equation for a constant input, costs one `exp`, and
    /// means a cooling engine follows `tau` to the last decimal instead of to
    /// whatever the frame rate allows.
    pub fn integrate(&mut self, dt: f64, heat_in: f64) {
        if !(dt.is_finite() && dt > 0.0 && heat_in.is_finite()) {
            return;
        }
        let target = self.equilibrium(heat_in);
        let decay = (-dt / self.time_constant()).exp();
        self.temperature = target + (self.temperature - target) * decay;
    }
}

// ---------------------------------------------------------------------------
// The cooling system
// ---------------------------------------------------------------------------

/// The wax thermostat and the radiator behind it.
///
/// A block with a fixed conductance to ambient has no operating temperature: it
/// settles wherever the load puts it, which for a V8 between idle and full
/// throttle is a spread of several hundred Kelvin. A real engine does not do
/// that, and the reason is one component. Below its opening temperature the
/// thermostat is shut and the only loss is what leaks off the outside of the
/// casting; above it the wax expands proportionally to how far past it the
/// coolant is, and the conductance climbs steeply enough that the plateau moves
/// by a couple of Kelvin across the whole load range.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Thermostat {
    /// Temperature at which the valve starts to crack open [K].
    pub open_temperature: f64,
    /// Loss straight off the outside of the block with the valve shut [W/K].
    pub bypass_conductance: f64,
    /// How fast the valve opens past its rating [W/K per K].
    pub gain: f64,
    /// Conductance with the valve wide open and the fan running [W/K].
    pub radiator_conductance: f64,
}

impl Default for Thermostat {
    /// An 88 C thermostat on a road-car cooling pack.
    fn default() -> Self {
        Self {
            open_temperature: 361.15,
            bypass_conductance: 22.0,
            gain: 200.0,
            radiator_conductance: 3_000.0,
        }
    }
}

impl Thermostat {
    /// Conductance from the block to ambient at a block temperature [W/K].
    pub fn conductance(&self, block_temperature: f64) -> f64 {
        let excess = (block_temperature - self.open_temperature).max(0.0);
        (self.bypass_conductance + self.gain * excess).min(self.radiator_conductance)
    }
}

// ---------------------------------------------------------------------------
// Oil
// ---------------------------------------------------------------------------

/// Vogel's law for the oil in the sump, and what it does to friction.
///
/// ```text
/// mu(T) = A exp( B / (T - C) )      [Pa s]
/// ```
///
/// Three constants rather than Arrhenius' two, because a two-constant fit is
/// badly wrong at both ends of the range an engine actually sees. The defaults
/// are fitted to a 10W-40 through its two published grade points, 95 cSt at
/// 40 C and 14 cSt at 100 C.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OilViscosity {
    /// Vogel `A`, the high-temperature asymptote [Pa s].
    pub a: f64,
    /// Vogel `B`, the activation term [K].
    pub b: f64,
    /// Vogel `C`, the pole the viscosity runs away at [K].
    pub c: f64,
    /// Temperature the friction correlation's coefficients were measured at [K].
    pub reference_temperature: f64,
    /// How hard friction follows viscosity [-].
    ///
    /// Not 1. A journal bearing shearing a fixed film would give Petroff's
    /// linear law, but an engine's rubbing surfaces sit in a mixed regime and
    /// the film thickness itself adjusts to the viscosity, which flattens the
    /// dependence a long way. Sandoval and Heywood's motoring-friction fit puts
    /// the exponent near a quarter, which is why a cold engine's friction is
    /// twice its warm value rather than the twenty times the viscosity ratio
    /// alone would suggest.
    pub exponent: f64,
}

impl Default for OilViscosity {
    /// A 10W-40 mineral oil against a correlation measured at 90 C.
    fn default() -> Self {
        Self {
            a: 6.899e-5,
            b: 1_161.8,
            c: 150.0,
            reference_temperature: 363.15,
            exponent: 0.24,
        }
    }
}

impl OilViscosity {
    /// Dynamic viscosity at a temperature [Pa s].
    pub fn dynamic_viscosity(&self, temperature: f64) -> f64 {
        // Below the Vogel pole the expression is meaningless and above it, far
        // enough down, it overflows; an oil that cold is a solid anyway.
        let above_pole = (temperature - self.c).max(20.0);
        self.a * (self.b / above_pole).exp()
    }

    /// Multiplier on the hydrodynamic friction terms at a temperature [-].
    ///
    /// `(mu(T) / mu(T_ref))^n`, which is 1 at the temperature the correlation
    /// was fitted at and rises as the oil thickens.
    pub fn friction_multiplier(&self, temperature: f64) -> f64 {
        let ratio = self.dynamic_viscosity(temperature)
            / self.dynamic_viscosity(self.reference_temperature);
        ratio.max(0.0).powf(self.exponent)
    }
}

// ---------------------------------------------------------------------------
// Pipe heat transfer
// ---------------------------------------------------------------------------

/// Heat capacity of a length of thin-walled steel tube [J/K].
pub fn pipe_wall_capacity(length: f64, diameter: f64) -> f64 {
    let volume = PI * diameter.max(1e-4) * PIPE_WALL_THICKNESS * length.max(1e-4);
    volume * PIPE_DENSITY * PIPE_SPECIFIC_HEAT
}

/// Gas-side film coefficient inside an exhaust pipe [W/(m^2 K)].
///
/// Dittus-Boelter for turbulent pipe flow,
///
/// ```text
/// Nu = 0.023 Re^0.8 Pr^0.4,     Re = 4 m_dot / (pi D mu)
/// ```
///
/// floored at the fully developed laminar value `Nu = 4.36`, which is what the
/// correlation is blended into below the transition and is also what keeps an
/// idling engine's pipes from going adiabatic. Scaled by
/// [`PULSATION_ENHANCEMENT`].
pub fn gas_film_coefficient(mass_flow: f64, diameter: f64) -> f64 {
    let d = diameter.max(1e-4);
    let reynolds = 4.0 * mass_flow.abs() / (PI * d * EXHAUST_VISCOSITY);
    let nusselt = (0.023 * reynolds.powf(0.8) * EXHAUST_PRANDTL.powf(0.4)).max(4.36);
    PULSATION_ENHANCEMENT * nusselt * EXHAUST_CONDUCTIVITY / d
}

/// Gas temperature leaving a pipe section whose wall sits at `wall` [K].
///
/// The steady constant-wall-temperature tube solution:
///
/// ```text
/// T_out = T_wall + (T_in - T_wall) exp( -h A / (m_dot c_p) )
/// ```
///
/// This is where the gradient Stage 10 wants comes from. It is not a decay
/// applied to the pipe, it is the exact answer for the section, and the heat it
/// says the wall took is what the wall is then warmed by — so a long pipe at low
/// flow cools its gas nearly to the wall and a short one at high flow barely
/// touches it.
pub fn outlet_temperature(
    inlet: f64,
    wall: f64,
    mass_flow: f64,
    film_coefficient: f64,
    surface_area: f64,
) -> f64 {
    let capacity_rate = mass_flow.abs() * EXHAUST_CP;
    if capacity_rate <= 1e-9 {
        // No flow, no convection: the gas in the pipe is simply the wall's.
        return wall;
    }
    let ntu = film_coefficient * surface_area / capacity_rate;
    wall + (inlet - wall) * (-ntu).exp()
}

/// Heat a stream gives up between two stations [W].
pub fn stream_heat(mass_flow: f64, inlet: f64, outlet: f64) -> f64 {
    mass_flow.abs() * EXHAUST_CP * (inlet - outlet)
}

/// Chamber wall temperature for a block at `block_temperature` [K].
///
/// `T_wall = T_block + Q / (K A)`: the drop across the head metal and the
/// coolant film, which is what stands between the surface Woschni is radiating
/// into and the bulk of the casting. `heat_flow` is one cylinder's mean wall
/// loss [W] and `area` its mean chamber surface [m^2].
pub fn chamber_wall_temperature(block_temperature: f64, heat_flow: f64, area: f64) -> f64 {
    let rise = heat_flow.max(0.0) / (HEAD_CONDUCTANCE * area.max(1e-6));
    (block_temperature + rise).min(MAX_WALL_TEMPERATURE)
}

// ---------------------------------------------------------------------------
// The exhaust, section by section
// ---------------------------------------------------------------------------

/// One pipe section's thermal state: the tube, and the gas leaving it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SectionThermal {
    /// The tube wall as a lumped mass.
    pub wall: ThermalMass,
    /// Gas temperature at the section's outlet [K].
    pub gas_outlet: f64,
    /// Bore of the section, kept for the film coefficient [m].
    pub diameter: f64,
    /// Gas-wetted internal surface area [m^2].
    pub area: f64,
}

impl SectionThermal {
    /// A section of thin-walled tube, everything at `temperature`.
    pub fn new(length: f64, diameter: f64, temperature: f64, ambient: f64) -> Self {
        let length = length.max(1e-3);
        let diameter = diameter.max(1e-4);
        let area = PI * diameter * length;
        // The outside of the tube is one wall thickness further out, and that is
        // the surface the still air under the car actually sees.
        let external = PI * (diameter + 2.0 * PIPE_WALL_THICKNESS) * length;
        Self {
            wall: ThermalMass::new(
                temperature,
                pipe_wall_capacity(length, diameter),
                PIPE_EXTERNAL_COEFFICIENT * external,
                ambient,
            ),
            gas_outlet: temperature,
            diameter,
            area,
        }
    }

    /// Advances the wall by one frame under a gas stream entering at `inlet`.
    ///
    /// Returns the gas temperature at the outlet, which is the next section's
    /// inlet. This is the whole of the gradient: each section takes what its own
    /// length, bore and flow let it take, and hands the rest downstream.
    pub fn integrate(&mut self, dt: f64, inlet: f64, mass_flow: f64) -> f64 {
        let film = gas_film_coefficient(mass_flow, self.diameter);
        self.gas_outlet =
            outlet_temperature(inlet, self.wall.temperature, mass_flow, film, self.area);
        let heat = stream_heat(mass_flow, inlet, self.gas_outlet);
        self.wall.integrate(dt, heat);
        self.gas_outlet
    }
}

/// Every pipe section of one exhaust system, in flow order.
///
/// The downstream chain is carried once rather than per bank: both banks of a
/// vee run the same collector outlet area, the same silencers and the same
/// tailpipe, and they pass the same mass flow through them, so their walls would
/// hold the same number twice.
#[derive(Debug, Clone, PartialEq)]
pub struct ExhaustThermal {
    /// One per primary runner, in cylinder order.
    pub primaries: Vec<SectionThermal>,
    /// Pipes between the collector and the silencers, in flow order.
    pub secondaries: Vec<SectionThermal>,
    /// The tailpipe.
    pub tailpipe: SectionThermal,
}

impl ExhaustThermal {
    /// Builds the sections of an exhaust system, each seeded at `seed(spec)`.
    pub(crate) fn build(
        exhaust: &ExhaustSystem,
        ambient: f64,
        seed: impl Fn(&PipeSection) -> f64,
    ) -> Self {
        let section = |spec: &PipeSection| {
            SectionThermal::new(spec.length, spec.diameter(), seed(spec), ambient)
        };
        Self {
            primaries: exhaust.primaries.iter().map(&section).collect(),
            secondaries: exhaust.secondary.iter().map(&section).collect(),
            tailpipe: section(&exhaust.tailpipe),
        }
    }

    /// An exhaust that has been run long enough to sit where its geometry says.
    ///
    /// The wall temperature on a [`PipeSection`] is the one the system was
    /// specified at, so a soaked system starts there and a run at that load
    /// leaves it there.
    pub fn soaked(exhaust: &ExhaustSystem, ambient: f64) -> Self {
        Self::build(exhaust, ambient, |spec| spec.wall_temperature)
    }

    /// An exhaust that has stood overnight: every section at ambient.
    pub fn cold(exhaust: &ExhaustSystem, ambient: f64) -> Self {
        Self::build(exhaust, ambient, |_| ambient)
    }

    /// Advances every section by one frame.
    ///
    /// `port_temperature` is the gas leaving the exhaust port,
    /// `cylinder_mass_flow` the mean exhaust flow of one cylinder, and
    /// `cylinders_per_bank` how many of those merge before the collector.
    pub fn integrate(
        &mut self,
        dt: f64,
        port_temperature: f64,
        cylinder_mass_flow: f64,
        cylinders_per_bank: usize,
    ) {
        let mut merged = 0.0;
        for primary in &mut self.primaries {
            merged += primary.integrate(dt, port_temperature, cylinder_mass_flow);
        }
        // Every bank's primaries pour into an identical collector, so what goes
        // downstream is the mean of what came out of them.
        let mut gas = if self.primaries.is_empty() {
            port_temperature
        } else {
            merged / self.primaries.len() as f64
        };

        let bank_flow = cylinder_mass_flow * cylinders_per_bank.max(1) as f64;
        for secondary in &mut self.secondaries {
            gas = secondary.integrate(dt, gas, bank_flow);
        }
        // Silencers sit between the last secondary and the tailpipe and take
        // heat out of the stream too, but they are a spec rather than a tube
        // here, so the tailpipe is handed the gas the pipework left.
        self.tailpipe.integrate(dt, gas, bank_flow);
    }

    /// Gas temperature in primary runner `i` [K].
    ///
    /// The outlet, which is what the section as a whole resonates at: the inlet
    /// is the port and the gradient between them is what this stage exists to
    /// produce.
    pub fn primary_gas(&self, cylinder: usize) -> f64 {
        match self.primaries.get(cylinder % self.primaries.len().max(1)) {
            Some(section) => section.gas_outlet,
            None => self.tailpipe.gas_outlet,
        }
    }

    /// Gas temperature arriving at the collector [K].
    pub fn collector_gas(&self) -> f64 {
        if self.primaries.is_empty() {
            return self.tailpipe.gas_outlet;
        }
        self.primaries.iter().map(|s| s.gas_outlet).sum::<f64>() / self.primaries.len() as f64
    }

    /// Gas temperature leaving the tailpipe [K].
    pub fn tailpipe_gas(&self) -> f64 {
        self.tailpipe.gas_outlet
    }
}

// ---------------------------------------------------------------------------
// The engine's thermal state
// ---------------------------------------------------------------------------

/// Every temperature in the engine that is not a gas state.
///
/// One body for the block, driven by the wall heat Woschni already computes plus
/// the work friction is turning into heat, and cooled through the thermostat.
/// The chamber wall the solver reads is derived from it rather than stored: it
/// is the block temperature plus the drop the current heat flux makes across the
/// head.
#[derive(Debug, Clone, PartialEq)]
pub struct EngineThermal {
    /// The block, head and the coolant and oil they carry.
    pub block: ThermalMass,
    /// The thermostat that decides where the block plateaus.
    pub thermostat: Thermostat,
    /// Every exhaust pipe section, each with a temperature of its own.
    pub exhaust: ExhaustThermal,
    /// Ambient air the whole engine eventually loses to [K].
    pub ambient: f64,
}

impl EngineThermal {
    /// A thermal state with the block at `temperature` and a given exhaust.
    fn with_block(
        block_mass: f64,
        ambient: f64,
        temperature: f64,
        exhaust: ExhaustThermal,
    ) -> Self {
        let thermostat = Thermostat::default();
        let capacity = WARM_UP_MASS_FRACTION * block_mass.max(1.0) * METAL_SPECIFIC_HEAT;
        Self {
            block: ThermalMass::new(
                temperature,
                capacity,
                thermostat.conductance(temperature),
                ambient,
            ),
            thermostat,
            exhaust,
            ambient,
        }
    }

    /// An engine that has been running long enough to be on its thermostat.
    ///
    /// The default, because a block built to be solved at a given speed is a
    /// block somebody wants steady-state numbers from. Starting cold is the
    /// deliberate act; see [`Self::cold`].
    pub fn soaked(block_mass: f64, exhaust: &ExhaustSystem, ambient: f64) -> Self {
        let open = Thermostat::default().open_temperature;
        Self::with_block(
            block_mass,
            ambient,
            open,
            ExhaustThermal::soaked(exhaust, ambient),
        )
    }

    /// An engine that has stood overnight: everything at ambient.
    pub fn cold(block_mass: f64, exhaust: &ExhaustSystem, ambient: f64) -> Self {
        Self::with_block(
            block_mass,
            ambient,
            ambient,
            ExhaustThermal::cold(exhaust, ambient),
        )
    }

    /// Re-sizes the block's capacity for a new dressed mass [kg].
    pub fn set_block_mass(&mut self, block_mass: f64) {
        self.block.capacity =
            (WARM_UP_MASS_FRACTION * block_mass.max(1.0) * METAL_SPECIFIC_HEAT).max(1e-3);
    }

    /// Rebuilds the exhaust sections for a new exhaust geometry.
    ///
    /// Each new section is seeded at where the engine's own warm-up says it
    /// should be — ambient on a cold engine, the temperature the geometry was
    /// specified at on a soaked one — so changing the exhaust part-way through a
    /// warm-up is not a step change in every resonance at once.
    pub fn rebuild_exhaust(&mut self, exhaust: &ExhaustSystem) {
        let warm = 1.0 - self.cold_fraction();
        let ambient = self.ambient;
        self.exhaust = ExhaustThermal::build(exhaust, ambient, |spec| {
            ambient + warm * (spec.wall_temperature - ambient)
        });
    }

    /// Advances the block and every pipe section by one frame.
    ///
    /// `chamber_heat` is the wall loss summed over every cylinder [W],
    /// `friction_heat` the power the crankshaft is spending on friction [W] —
    /// all of which ends up in the oil and the bearings and from there in the
    /// block — and `port_temperature` and `cylinder_mass_flow` are the stream
    /// one cylinder is pushing into its primary.
    pub fn integrate(
        &mut self,
        dt: f64,
        chamber_heat: f64,
        friction_heat: f64,
        port_temperature: f64,
        cylinder_mass_flow: f64,
        cylinders_per_bank: usize,
    ) {
        self.block.sink_temperature = self.ambient;
        self.block.conductance = self.thermostat.conductance(self.block.temperature);
        self.block
            .integrate(dt, chamber_heat.max(0.0) + friction_heat.max(0.0));
        self.exhaust
            .integrate(dt, port_temperature, cylinder_mass_flow, cylinders_per_bank);
    }

    /// Temperature of the oil the bearings are shearing [K].
    ///
    /// The block's, because they are the same body here. The sump does run a
    /// little behind the coolant on a warm-up and a little ahead of it under
    /// load, but a second lumped mass to carry that difference would move the
    /// friction by a couple of percent for a minute and be inaudible.
    pub fn oil_temperature(&self) -> f64 {
        self.block.temperature
    }

    /// Block metal temperature [K].
    pub fn block_temperature(&self) -> f64 {
        self.block.temperature
    }

    /// Chamber wall temperature the solver should run against [K].
    ///
    /// `chamber_heat` is one cylinder's mean wall loss [W] and `area` its
    /// cycle-mean wetted surface [m^2].
    pub fn wall_temperature(&self, chamber_heat: f64, area: f64) -> f64 {
        chamber_wall_temperature(self.block.temperature, chamber_heat, area)
    }

    /// Coolant bulk temperature [K].
    pub fn coolant_temperature(&self) -> f64 {
        self.block.temperature
    }

    /// Cylinder head metal temperature [K].
    pub fn head_temperature(&self) -> f64 {
        self.block.temperature + (1.0 - self.cold_fraction()) * 8.0
    }

    /// Oil gallery pressure [Pa] driven by crankshaft-driven pump with relief valve.
    pub fn oil_pressure(&self, rpm: f64) -> f64 {
        if rpm <= 0.0 {
            return 0.0;
        }
        let visc = OilViscosity::default().friction_multiplier(self.oil_temperature());
        let speed_ratio = (rpm / 1000.0).max(0.1);
        let dynamic_bar = (1.5 + speed_ratio * 0.8) * visc.sqrt();
        let relief_bar = 5.5;
        let p_bar = dynamic_bar.min(relief_bar);
        p_bar * 100_000.0
    }

    /// How cold the engine still is, `1` at ambient and `0` on the thermostat [-].
    ///
    /// What a fast-idle schedule and a warm-up enrichment are both written
    /// against: one number saying how far through the warm-up the engine is.
    pub fn cold_fraction(&self) -> f64 {
        let span = (self.thermostat.open_temperature - self.ambient).max(1.0);
        ((self.thermostat.open_temperature - self.block.temperature) / span).clamp(0.0, 1.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx(a: f64, b: f64, tol: f64) {
        assert!((a - b).abs() <= tol, "{a} != {b} (tol {tol})");
    }

    #[test]
    fn a_mass_settles_at_its_equilibrium() {
        let mut body = ThermalMass::new(300.0, 1_000.0, 10.0, 300.0);
        // Ten time constants of 100 s each.
        for _ in 0..100_000 {
            body.integrate(0.01, 500.0);
        }
        // T_sink + Q/h = 300 + 50.
        approx(body.temperature, 350.0, 1e-2);
    }

    #[test]
    fn cooling_follows_the_modelled_time_constant() {
        let mut body = ThermalMass::new(1_000.0, 2_000.0, 20.0, 300.0);
        let tau = body.time_constant();
        approx(tau, 100.0, 1e-9);

        let start = body.temperature;
        let steps = 1_000;
        for _ in 0..steps {
            body.integrate(tau / steps as f64, 0.0);
        }
        // One time constant leaves 1/e of the excess over the sink.
        approx(
            body.temperature - 300.0,
            (start - 300.0) / std::f64::consts::E,
            1e-6,
        );
    }

    #[test]
    fn the_thermostat_pins_the_plateau_across_the_load_range() {
        let stat = Thermostat::default();
        let plateau = |heat: f64| {
            let mut block = ThermalMass::new(293.15, 22_000.0, stat.bypass_conductance, 293.15);
            for _ in 0..200_000 {
                block.conductance = stat.conductance(block.temperature);
                block.integrate(0.01, heat);
            }
            block.temperature
        };
        let idle = plateau(8_000.0);
        let loaded = plateau(60_000.0);
        assert!(
            idle > stat.open_temperature && idle < stat.open_temperature + 5.0,
            "idle plateau {idle} is not on the thermostat"
        );
        assert!(
            loaded - idle < 5.0,
            "plateau moved {:.1} K from idle to full load",
            loaded - idle
        );
    }

    #[test]
    fn vogel_viscosity_falls_with_temperature() {
        let oil = OilViscosity::default();
        let cold = oil.dynamic_viscosity(293.15);
        let warm = oil.dynamic_viscosity(373.15);
        assert!(cold > warm, "oil must thin as it warms");
        // The grade points it was fitted through, in Pa s.
        approx(oil.dynamic_viscosity(313.15), 0.0855, 1e-3);
        approx(oil.dynamic_viscosity(373.15), 0.0126, 1e-3);
    }

    #[test]
    fn the_friction_multiplier_is_unity_at_the_reference() {
        let oil = OilViscosity::default();
        approx(
            oil.friction_multiplier(oil.reference_temperature),
            1.0,
            1e-12,
        );
        let cold = oil.friction_multiplier(293.15);
        assert!(
            (1.5..3.0).contains(&cold),
            "a cold engine's hydrodynamic friction multiplier is {cold}, not near two"
        );
    }

    #[test]
    fn a_pipe_cools_its_gas_towards_the_wall() {
        let area = PI * 0.040 * 0.45;
        let h = gas_film_coefficient(0.0135, 0.040);
        let out = outlet_temperature(1_100.0, 900.0, 0.0135, h, area);
        assert!(
            out < 1_100.0 && out > 900.0,
            "outlet {out} left the bracket"
        );
        // Halve the flow and the same pipe takes more out of it.
        let slower = outlet_temperature(1_100.0, 900.0, 0.0068, h, area);
        assert!(slower < out, "less flow must be cooled further");
    }

    #[test]
    fn a_stalled_pipe_holds_its_wall_temperature() {
        approx(
            outlet_temperature(1_100.0, 900.0, 0.0, 50.0, 0.05),
            900.0,
            0.0,
        );
    }

    #[test]
    fn the_chamber_wall_sits_above_the_block() {
        let area = 2.0 * PI * 0.043 * 0.043;
        let warm = chamber_wall_temperature(363.15, 1_000.0, area);
        assert!(
            (430.0..470.0).contains(&warm),
            "a warm chamber wall at idle is {warm} K, not the 450 K this used to assume"
        );
        assert!(
            chamber_wall_temperature(363.15, 1e9, area) <= MAX_WALL_TEMPERATURE,
            "the head must not be allowed past its material limit"
        );
    }

    #[test]
    fn oil_pressure_scales_with_rpm_and_viscosity() {
        let thermal =
            EngineThermal::soaked(150.0, &ExhaustSystem::default_for_cylinders(4, 1), 293.15);
        assert_eq!(thermal.oil_pressure(0.0), 0.0);
        let p_idle = thermal.oil_pressure(850.0);
        let p_mid = thermal.oil_pressure(3_500.0);
        assert!(
            p_idle > 100_000.0,
            "idle oil pressure must exceed 1 bar: {p_idle} Pa"
        );
        assert!(p_mid > p_idle, "oil pressure must rise with engine rpm");
        assert!(
            p_mid <= 560_000.0,
            "oil pressure relief valve must cap at ~5.5 bar: {p_mid} Pa"
        );
    }
}
