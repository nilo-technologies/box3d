// SPDX-FileCopyrightText: 2026 Nilo Technologies
// SPDX-License-Identifier: MIT

#pragma once

#include "base.h"
#include "id.h"
#include "math_functions.h"
#include "types.h"

#include <stdbool.h>
#include <stdint.h>

/**
 * @defgroup vehicle Vehicle
 * A wheeled vehicle controller. The vehicle drives an existing dynamic chassis body. Each
 * wheel casts against the world once per step and is solved as a soft suspension plus tire
 * constraint in every substep. Engine, clutch, gearbox and differentials are solved implicitly
 * once per step.
 *
 * Conventions: wheel positions, forward and up are in the chassis body's local frame.
 * Steering input is positive to the right. Wheel spin is positive when rolling forward.
 *
 * Vehicles are not supported in recorded worlds or snapshots.
 * @{
 */

/// Maximum number of wheels on one vehicle
#define B3_MAX_VEHICLE_WHEELS 8

/// Maximum number of forward gears
#define B3_MAX_VEHICLE_GEARS 8

/// Maximum number of differentials
#define B3_MAX_VEHICLE_DIFFERENTIALS 4

/// Maximum number of anti-roll bars
#define B3_MAX_VEHICLE_ANTI_ROLL_BARS 4

/// How a wheel probes the ground
typedef enum b3VehicleCastType
{
	/// Rounded cylinder matching the wheel radius and width. Does not fall into cracks.
	b3_vehicleCastCylinder,

	/// Sphere with the wheel radius
	b3_vehicleCastSphere,

	/// Single ray through the wheel center. Cheapest.
	b3_vehicleCastRay,
} b3VehicleCastType;

/// Suspension settings for one wheel. Lengths are measured from the attachment point to the
/// wheel center along the chassis down axis.
typedef struct b3VehicleSuspension
{
	/// Length at full compression, where the bump stop engages (m)
	float minLength;

	/// Length at full extension (m)
	float maxLength;

	/// Extra spring length that pushes the car up without changing the stiffness (m)
	float preload;

	/// Ride frequency, computed against the wheel's sprung mass (Hz)
	float frequency;

	/// Damping ratio while compressing
	float compressionDamping;

	/// Damping ratio while extending
	float reboundDamping;

	/// Hard stop at minLength so hard landings do not sink the wheel through the chassis
	bool enableBumpStop;
} b3VehicleSuspension;

/// Tire friction. Each axis is a 3 point curve: zero at no slip, rising to the peak, then
/// falling to the slide value. The coefficients are multiplied by the ground friction relative
/// to the default shape friction (0.6).
typedef struct b3VehicleTire
{
	/// Slip ratio of peak longitudinal grip
	float longitudinalPeakSlip;

	/// Peak longitudinal friction coefficient
	float longitudinalPeakFriction;

	/// Slip ratio where the tire is fully sliding
	float longitudinalSlideSlip;

	/// Fully sliding longitudinal friction coefficient
	float longitudinalSlideFriction;

	/// Slip angle of peak lateral grip (radians)
	float lateralPeakAngle;

	/// Peak lateral friction coefficient
	float lateralPeakFriction;

	/// Slip angle where the tire is fully sliding (radians)
	float lateralSlideAngle;

	/// Fully sliding lateral friction coefficient
	float lateralSlideFriction;

	/// Rolling resistance coefficient, a torque of coefficient * load * radius opposing spin
	float rollingResistance;

	/// Distance the tire rolls for its slip ratio to build up (m). At low speed a small difference
	/// between the wheel and ground speed then takes a moment to become a large slip, so pulling
	/// away does not break the tire loose at once. At speed it settles within a step. 0 is off.
	float longitudinalRelaxationLength;
} b3VehicleTire;

/// Wheel definition
typedef struct b3VehicleWheelDef
{
	/// Suspension attachment point in chassis local space
	b3Vec3 position;

	/// Wheel radius (m)
	float radius;

	/// Wheel width (m)
	float width;

	/// Spin inertia of the wheel and brake disc (kg*m^2)
	float inertia;

	/// Multiplier on the vehicle steer angle. 1 for front wheels, 0 for fixed wheels, negative for
	/// rear steering.
	float steerFactor;

	/// Brake torque at full brake input (N*m)
	float maxBrakeTorque;

	/// Brake torque at full handbrake input (N*m)
	float maxHandBrakeTorque;

	/// Spin damping (N*m*s/rad)
	float angularDamping;

	/// Ground probe shape
	b3VehicleCastType castType;

	b3VehicleSuspension suspension;
	b3VehicleTire tire;
} b3VehicleWheelDef;

/// Chassis settings
typedef struct b3VehicleChassis
{
	/// Chassis mass (kg). Zero or less keeps the mass computed from the body's shapes.
	float mass;

	/// Place the center of mass at the resting axle height. Lowering the center of mass is the
	/// most effective way to stop a car from flipping.
	bool autoCenterOfMass;

	/// Added to the center of mass, local space (m)
	b3Vec3 centerOfMassOffset;

	/// Forward axis in chassis local space
	b3Vec3 forward;

	/// Up axis in chassis local space. Suspensions extend along the opposite direction.
	b3Vec3 up;

	/// Ground normals steeper than this are ignored by the wheels (radians)
	float maxSlopeAngle;

	/// Filter for the wheel casts. The chassis's own shapes are always ignored.
	b3QueryFilter castFilter;
} b3VehicleChassis;

/// Engine settings. The torque curve is linear from idle to the peak and from the peak to the
/// max rpm.
typedef struct b3VehicleEngine
{
	/// Peak torque (N*m)
	float maxTorque;

	/// Rpm of peak torque
	float peakTorqueRpm;

	/// Fraction of the peak torque available at idle
	float idleTorqueScale;

	/// Fraction of the peak torque available at the max rpm
	float maxRpmTorqueScale;

	/// Idle rpm. The engine never drops below it.
	float idleRpm;

	/// Rev limiter rpm
	float maxRpm;

	/// Engine and flywheel inertia (kg*m^2)
	float inertia;

	/// Internal friction, gives engine braking (N*m*s/rad)
	float angularDamping;
} b3VehicleEngine;

/// Maximum number of turbo stages
#define B3_MAX_VEHICLE_TURBO_STAGES 2

/// One compressor stage. Its shaft spools toward a target set by the pedal and the engine rpm and
/// spins down with the bleed time. The boost it delivers is what the shaft makes, capped by what
/// the pedal asks for. A stage with zero gain is off.
typedef struct b3VehicleTurboStage
{
	/// Torque added at full boost, as a fraction of the torque curve. 0.5 gives 1.5 times the torque.
	float gain;

	/// Rpm where boost starts to build
	float spoolRpm;

	/// Rpm where the stage reaches full boost. The boost target eases in between the two.
	float fullBoostRpm;

	/// Turbo lag: time constant of the shaft spooling up at full load and the full boost rpm (s).
	/// Exhaust energy drives the shaft, so the lag is longer below that rpm or at part load and
	/// shorter above it, and the spool up gathers pace as boost builds.
	float spoolTime;

	/// Time constant of the shaft spinning down off throttle (s). The boost itself dumps through the
	/// blow-off valve within about 0.1 s, but a shaft still spinning brings it back as soon as the
	/// throttle opens, so a longer bleed time keeps more boost through shifts and short lifts.
	float bleedTime;
} b3VehicleTurboStage;

/// Forced induction. The torque curve is multiplied by 1 plus the summed boost of the stages. All
/// zero, as in b3DefaultVehicleDef, is a naturally aspirated engine.
///
/// The pedal asks for a share of the fully boosted torque, (1 + the summed gains) times the curve.
/// The engine makes the first 1 / (1 + gains) of the pedal with the throttle plate alone, so boost
/// only builds past that point. The boost built so far caps the torque.
///
/// A single turbo uses one stage. A sequential twin turbo uses a small fast stage at low rpm and a
/// large slow stage higher up. A supercharger has no lag: an instant fraction of 1 and a parasitic
/// torque for the belt drive.
typedef struct b3VehicleTurbo
{
	b3VehicleTurboStage stages[B3_MAX_VEHICLE_TURBO_STAGES];

	/// Fraction of the shaft spool that follows its target with no lag. 0 for an exhaust driven
	/// turbo, between 0 and 1 for an electrically assisted turbo, 1 for a supercharger.
	float instantFraction;

	/// Torque the engine spends driving the compressor at full boost, scaled by the boost (N*m)
	float parasiticTorque;

	/// Anti-lag: the shaft and the manifold pressure ignore the pedal, so boost holds off throttle
	/// and through shifts. They still follow the rpm.
	bool antiLag;
} b3VehicleTurbo;

typedef enum b3VehicleTransmissionMode
{
	/// Shifts by rpm. Pressing reverse while stopped selects reverse. See also
	/// b3VehicleAssists::brakeReverse.
	b3_vehicleTransmissionAuto,

	/// Use b3Vehicle_SetGear
	b3_vehicleTransmissionManual,
} b3VehicleTransmissionMode;

/// Gearbox and clutch
typedef struct b3VehicleTransmission
{
	b3VehicleTransmissionMode mode;

	/// Forward gear ratios, first gear first
	float gearRatios[B3_MAX_VEHICLE_GEARS];

	/// Number of forward gears in use
	int gearCount;

	/// Reverse gear ratio (positive)
	float reverseRatio;

	/// Final drive ratio applied to all gears
	float finalDrive;

	/// Automatic upshift rpm
	float shiftUpRpm;

	/// Automatic downshift rpm
	float shiftDownRpm;

	/// Time with the clutch open and the throttle cut during a shift (s)
	float shiftTime;

	/// Time for the clutch to re-engage after a shift (s)
	float clutchReleaseTime;

	/// Minimum time between automatic shifts (s)
	float shiftLatency;

	/// Clutch coupling strength (N*m*s/rad)
	float clutchStrength;
} b3VehicleTransmission;

/// A differential splits engine torque between two wheels
typedef struct b3VehicleDifferential
{
	/// Wheel index or -1
	int leftWheel;

	/// Wheel index or -1
	int rightWheel;

	/// Share of the engine torque this differential receives. Shares should sum to 1.
	float torqueRatio;

	/// Share of this differential's torque sent to the left wheel
	float leftRightSplit;

	/// Max wheel speed ratio before torque moves to the slower wheel. 1 or less is an open
	/// differential.
	float limitedSlipRatio;
} b3VehicleDifferential;

/// Drivetrain layout
typedef struct b3VehicleDrivetrain
{
	b3VehicleDifferential differentials[B3_MAX_VEHICLE_DIFFERENTIALS];
	int differentialCount;

	/// Limited slip between differentials. 1 or less is open.
	float centerLimitedSlipRatio;
} b3VehicleDrivetrain;

/// Anti-roll bar connecting two wheels. Pushes on the compression difference.
typedef struct b3VehicleAntiRollBar
{
	int leftWheel;
	int rightWheel;

	/// Stiffness (N/m)
	float stiffness;
} b3VehicleAntiRollBar;

typedef struct b3VehicleAntiRoll
{
	b3VehicleAntiRollBar bars[B3_MAX_VEHICLE_ANTI_ROLL_BARS];
	int barCount;
} b3VehicleAntiRoll;

/// Steering
typedef struct b3VehicleSteering
{
	/// Steer angle at rest (radians)
	float maxAngle;

	/// Speed where the max angle is halved, it falls off with speed squared. Zero disables. (m/s)
	float speedReference;

	/// Rate toward a larger steer angle (rad/s)
	float steerSpeed;

	/// Rate back toward center (rad/s)
	float returnSpeed;

	/// 0 parallel steering, 1 full Ackermann geometry
	float ackermann;
} b3VehicleSteering;

/// Driving assists and arcade shortcuts
typedef struct b3VehicleAssists
{
	/// Steers into a rear slide, as a fraction of the slide angle
	float counterSteer;

	/// Lateral grip multiplier on handbraked wheels at full handbrake. Below 1 makes drifting easy.
	float driftGripScale;

	/// Limits the throttle so no grounded driven wheel gets more torque than its tire can transmit
	/// at peak grip. On wheels behind the center of mass it also leaves room for the cornering
	/// force, so full throttle mid corner does not spin the car.
	bool tractionControl;

	/// Caps the brake torque at what the tire can transmit at peak grip so the wheels do not lock
	bool abs;

	/// Downforce coefficient, force = downforce * speed^2 (N*s^2/m^2)
	float downforce;

	/// Aerodynamic drag coefficient, force = drag * speed^2 (N*s^2/m^2)
	float drag;

	/// Raises the point where lateral tire forces act toward the center of mass height.
	/// 0 is physical, 1 removes all roll from cornering.
	float rollInfluence;

	/// Angular acceleration from input while all wheels are airborne (rad/s^2)
	float airControl;

	/// Angular damping while all wheels are airborne (1/s)
	float airAngularDamping;

	/// Roll-only righting stiffness while partly grounded or stuck on the side or roof (1/s^2)
	float selfRighting;

	/// Limits tilt from the world up axis (radians). 0 disables.
	float maxTiltAngle;

	/// Holds the car still at low speed with no throttle
	bool stickyTire;

	/// Speed below which the sticky tire engages (m/s)
	float stickySpeed;

	/// Time below the sticky speed before it engages (s)
	float stickyTime;

	/// With the automatic gearbox and no forward input, the brake selects reverse and drives
	/// backward once stopped. Off, the brake only brakes and holds the car still.
	bool brakeReverse;

	/// Pulling away from a standstill, holds the driven tires at their peak slip so full throttle
	/// neither spins them up nor leaves grip unused. Works with traction control on or off, and
	/// ends when the throttle is released or the tires grip at full throttle with room to spare.
	bool launchControl;
} b3VehicleAssists;

/// Vehicle definition. Use b3DefaultVehicleDef to initialize. The default is a 1300 kg rear
/// wheel drive sedan with 4 wheels in the order front left, front right, rear left, rear right.
typedef struct b3VehicleDef
{
	/// Existing dynamic body used as the chassis. Create its shapes before the vehicle.
	b3BodyId chassisId;

	b3VehicleChassis chassis;
	b3VehicleWheelDef wheels[B3_MAX_VEHICLE_WHEELS];
	int wheelCount;
	b3VehicleEngine engine;
	b3VehicleTurbo turbo;
	b3VehicleTransmission transmission;
	b3VehicleDrivetrain drivetrain;
	b3VehicleAntiRoll antiRoll;
	b3VehicleSteering steering;
	b3VehicleAssists assists;

	void* userData;

	/// Used internally to detect a valid definition. DO NOT SET.
	int internalValue;
} b3VehicleDef;

/// Driver input
typedef struct b3VehicleInput
{
	/// -1 full reverse, 1 full forward. With the automatic gearbox, the opposite direction brakes
	/// first and selects reverse once stopped.
	float forward;

	/// 0 to 1. With the automatic gearbox and b3VehicleAssists::brakeReverse, it also selects
	/// reverse once stopped when there is no forward input.
	float brake;

	/// -1 full left, 1 full right
	float steer;

	/// 0 to 1
	float handbrake;
} b3VehicleInput;

/// Vehicle readout, updated each step
typedef struct b3VehicleState
{
	/// Forward speed (m/s)
	float speed;

	float engineRpm;

	/// Summed turbo boost. The engine makes 1 + boost times its torque curve.
	float boost;

	/// -1 reverse, 0 neutral, 1 and up forward gears
	int gear;

	bool isShifting;

	/// Throttle and brake after the automatic gearbox logic
	float throttle;
	float brake;

	/// Center steer angle, positive left (radians)
	float steerAngle;

	int wheelsInContact;

	/// Upside down or on its side and nearly stopped for 2 seconds. See b3Vehicle_Reset.
	bool stuck;

	/// Launch control is managing the drive torque
	bool isLaunching;
} b3VehicleState;

/// Wheel readout, updated each step
typedef struct b3VehicleWheelState
{
	bool hasContact;

	b3Pos contactPoint;
	b3Vec3 contactNormal;
	b3ShapeId groundShapeId;
	uint64_t userMaterialId;

	/// Attachment point to wheel center (m)
	float suspensionLength;

	/// 0 fully extended, 1 at the bump stop
	float compression;

	/// Normal force (N)
	float load;

	/// Forward tire force on the chassis (N)
	float longitudinalForce;

	/// Lateral tire force on the chassis (N)
	float lateralForce;

	float slipRatio;

	/// Radians
	float slipAngle;

	/// Spin (rad/s)
	float angularVelocity;

	/// Accumulated spin angle for visuals (radians)
	float rotationAngle;

	/// Steer angle, positive left (radians)
	float steerAngle;

	float driveTorque;
	float brakeTorque;

	/// The sticky tire is holding this wheel
	bool isSticky;
} b3VehicleWheelState;

/// Bits in b3VehicleWheelData::flags
typedef enum b3VehicleWheelFlags
{
	b3_vehicleWheelContact = 0x1,
	b3_vehicleWheelSticky = 0x2,
} b3VehicleWheelFlags;

/// Wheel state and transform packed for bulk reads, see b3Vehicle_GetWheelData. Every field is
/// made of 4 byte words, and the only 8 byte fields, groundShapeId and userMaterialId, come last,
/// so the record has no padding and a fixed 120 byte (30 word) layout. Words are little endian, as on wasm, x86 and ARM. From JavaScript,
/// view the output with a Float32Array for the floats and a Uint32Array over the same memory for
/// the flags and ids. The comments give the word offset of each field. Without contact the
/// contact point, normal, ground shape and material are zero.
typedef struct b3VehicleWheelData
{
	/// [0] Wheel center in world space, as b3Vehicle_GetWheelTransform. Rounded to float in
	/// double precision builds.
	b3Vec3 position;

	/// [3] Wheel rotation in world space as x, y, z, w
	b3Quat rotation;

	/// [7] Contact point in world space, rounded to float in double precision builds
	b3Vec3 contactPoint;

	/// [10]
	b3Vec3 contactNormal;

	/// [13] Attachment point to wheel center (m)
	float suspensionLength;

	/// [14] 0 fully extended, 1 at the bump stop
	float compression;

	/// [15] Normal force (N)
	float load;

	/// [16] Forward tire force on the chassis (N)
	float longitudinalForce;

	/// [17] Lateral tire force on the chassis (N)
	float lateralForce;

	/// [18]
	float slipRatio;

	/// [19] Radians
	float slipAngle;

	/// [20] Spin (rad/s)
	float angularVelocity;

	/// [21] Accumulated spin angle for visuals (radians)
	float rotationAngle;

	/// [22] Steer angle, positive left (radians)
	float steerAngle;

	/// [23]
	float driveTorque;

	/// [24]
	float brakeTorque;

	/// [25] b3VehicleWheelFlags
	uint32_t flags;

	/// [26] index1 as int32, then [27] world0 in the low 16 bits and generation in the high 16 bits
	b3ShapeId groundShapeId;

	/// [28] low 32 bits, [29] high 32 bits
	uint64_t userMaterialId;
} b3VehicleWheelData;

B3_API b3VehicleDef b3DefaultVehicleDef( void );
B3_API b3VehicleWheelDef b3DefaultVehicleWheelDef( void );

/// Create a vehicle on an existing dynamic chassis body. Destroying the chassis destroys the
/// vehicle.
B3_API b3VehicleId b3CreateVehicle( b3WorldId worldId, const b3VehicleDef* def );
B3_API void b3DestroyVehicle( b3VehicleId vehicleId );
B3_API bool b3Vehicle_IsValid( b3VehicleId vehicleId );
B3_API b3BodyId b3Vehicle_GetChassisBody( b3VehicleId vehicleId );
B3_API int b3Vehicle_GetWheelCount( b3VehicleId vehicleId );

B3_API void b3Vehicle_SetInput( b3VehicleId vehicleId, b3VehicleInput input );
B3_API b3VehicleInput b3Vehicle_GetInput( b3VehicleId vehicleId );

/// Select a gear: -1 reverse, 0 neutral, 1 and up forward. Used by the manual gearbox, the
/// automatic gearbox may change it again.
B3_API void b3Vehicle_SetGear( b3VehicleId vehicleId, int gear );

/// Teleport the chassis, clear its velocity and reset the drivetrain
B3_API void b3Vehicle_Reset( b3VehicleId vehicleId, b3Pos position, b3Quat rotation );

/// Live settings. Changes take effect on the next step.
B3_API void b3Vehicle_SetChassis( b3VehicleId vehicleId, b3VehicleChassis chassis );
B3_API b3VehicleChassis b3Vehicle_GetChassis( b3VehicleId vehicleId );
B3_API void b3Vehicle_SetWheel( b3VehicleId vehicleId, int wheelIndex, b3VehicleWheelDef wheel );
B3_API b3VehicleWheelDef b3Vehicle_GetWheel( b3VehicleId vehicleId, int wheelIndex );
B3_API void b3Vehicle_SetEngine( b3VehicleId vehicleId, b3VehicleEngine engine );
B3_API b3VehicleEngine b3Vehicle_GetEngine( b3VehicleId vehicleId );
B3_API void b3Vehicle_SetTurbo( b3VehicleId vehicleId, b3VehicleTurbo turbo );
B3_API b3VehicleTurbo b3Vehicle_GetTurbo( b3VehicleId vehicleId );
B3_API void b3Vehicle_SetTransmission( b3VehicleId vehicleId, b3VehicleTransmission transmission );
B3_API b3VehicleTransmission b3Vehicle_GetTransmission( b3VehicleId vehicleId );
B3_API void b3Vehicle_SetDrivetrain( b3VehicleId vehicleId, b3VehicleDrivetrain drivetrain );
B3_API b3VehicleDrivetrain b3Vehicle_GetDrivetrain( b3VehicleId vehicleId );
B3_API void b3Vehicle_SetAntiRoll( b3VehicleId vehicleId, b3VehicleAntiRoll antiRoll );
B3_API b3VehicleAntiRoll b3Vehicle_GetAntiRoll( b3VehicleId vehicleId );
B3_API void b3Vehicle_SetSteering( b3VehicleId vehicleId, b3VehicleSteering steering );
B3_API b3VehicleSteering b3Vehicle_GetSteering( b3VehicleId vehicleId );
B3_API void b3Vehicle_SetAssists( b3VehicleId vehicleId, b3VehicleAssists assists );
B3_API b3VehicleAssists b3Vehicle_GetAssists( b3VehicleId vehicleId );

B3_API void b3Vehicle_SetUserData( b3VehicleId vehicleId, void* userData );
B3_API void* b3Vehicle_GetUserData( b3VehicleId vehicleId );

B3_API b3VehicleState b3Vehicle_GetState( b3VehicleId vehicleId );
B3_API b3VehicleWheelState b3Vehicle_GetWheelState( b3VehicleId vehicleId, int wheelIndex );

/// Wheel center and orientation in world space, including steering and spin. The wheel's
/// axle is the chassis local axis up x forward.
B3_API b3WorldTransform b3Vehicle_GetWheelTransform( b3VehicleId vehicleId, int wheelIndex );

/// Writes the state and world transform of every wheel into data, in wheel order, in one call.
/// Writes at most capacity records and returns the number written, 0 for a negative capacity.
B3_API int b3Vehicle_GetWheelData( b3VehicleId vehicleId, b3VehicleWheelData* data, int capacity );

/// Sprung mass carried by a wheel's suspension (kg)
B3_API float b3Vehicle_GetSprungMass( b3VehicleId vehicleId, int wheelIndex );

/// Number of vehicles in the world
B3_API int b3World_GetVehicleCount( b3WorldId worldId );

/**@}*/ // vehicle
