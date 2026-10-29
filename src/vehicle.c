// SPDX-FileCopyrightText: 2026 Nilo Technologies
// SPDX-License-Identifier: MIT

#include "vehicle.h"

#include "body.h"
#include "core.h"
#include "id_pool.h"
#include "joint.h"
#include "physics_world.h"
#include "shape.h"
#include "solver_set.h"

// needed for dll export
#include "box3d/box3d.h"

#include <float.h>
#include <stddef.h>

#define B3_VEHICLE_REFERENCE_FRICTION 0.6f
#define B3_RPM_TO_RADIANS ( 2.0f * B3_PI / 60.0f )
#define B3_RADIANS_TO_RPM ( 60.0f / ( 2.0f * B3_PI ) )
#define B3_VEHICLE_RING_COUNT 12

// cos and sin at 30 degree increments, exact so the cast shape is the same on every platform
static const float b3_ringCos[B3_VEHICLE_RING_COUNT] = { 1.0f,	0.8660254f,	 0.5f, 0.0f, -0.5f, -0.8660254f,
														 -1.0f, -0.8660254f, -0.5f, 0.0f, 0.5f,	 0.8660254f };
static const float b3_ringSin[B3_VEHICLE_RING_COUNT] = { 0.0f,	0.5f,  0.8660254f,	1.0f,  0.8660254f,	0.5f,
														 0.0f, -0.5f, -0.8660254f, -1.0f, -0.8660254f, -0.5f };

static float b3DegToRad( float degrees )
{
	return degrees * ( B3_PI / 180.0f );
}

b3VehicleWheelDef b3DefaultVehicleWheelDef( void )
{
	b3VehicleWheelDef def = { 0 };
	def.radius = 0.34f;
	def.width = 0.22f;
	def.inertia = 1.1f;
	def.steerFactor = 0.0f;
	def.maxBrakeTorque = 1500.0f;
	def.maxHandBrakeTorque = 0.0f;
	def.angularDamping = 0.2f;
	def.castType = b3_vehicleCastCylinder;

	def.suspension.minLength = 0.2f;
	def.suspension.maxLength = 0.4f;
	def.suspension.preload = 0.0f;
	def.suspension.frequency = 1.8f;
	def.suspension.compressionDamping = 0.5f;
	def.suspension.reboundDamping = 0.8f;
	def.suspension.enableBumpStop = true;

	def.tire.longitudinalPeakSlip = 0.1f;
	def.tire.longitudinalPeakFriction = 1.2f;
	def.tire.longitudinalSlideSlip = 0.4f;
	def.tire.longitudinalSlideFriction = 0.96f;
	def.tire.lateralPeakAngle = b3DegToRad( 7.0f );
	def.tire.lateralPeakFriction = 1.15f;
	def.tire.lateralSlideAngle = b3DegToRad( 20.0f );
	def.tire.lateralSlideFriction = 0.92f;
	def.tire.rollingResistance = 0.015f;
	return def;
}

b3VehicleDef b3DefaultVehicleDef( void )
{
	b3VehicleDef def = { 0 };

	def.chassis.mass = 1300.0f;
	def.chassis.autoCenterOfMass = true;
	def.chassis.centerOfMassOffset = b3Vec3_zero;
	def.chassis.forward = (b3Vec3){ 0.0f, 0.0f, 1.0f };
	def.chassis.up = (b3Vec3){ 0.0f, 1.0f, 0.0f };
	def.chassis.maxSlopeAngle = b3DegToRad( 80.0f );
	def.chassis.castFilter = b3DefaultQueryFilter();

	// Left is +x when forward is +z and up is +y
	b3Vec3 positions[4] = {
		{ 0.78f, -0.05f, 1.3f },
		{ -0.78f, -0.05f, 1.3f },
		{ 0.78f, -0.05f, -1.3f },
		{ -0.78f, -0.05f, -1.3f },
	};

	def.wheelCount = 4;
	for ( int i = 0; i < 4; ++i )
	{
		b3VehicleWheelDef* wheel = def.wheels + i;
		*wheel = b3DefaultVehicleWheelDef();
		wheel->position = positions[i];

		bool front = i < 2;
		wheel->steerFactor = front ? 1.0f : 0.0f;
		wheel->maxBrakeTorque = front ? 2000.0f : 1200.0f;
		wheel->maxHandBrakeTorque = front ? 0.0f : 3000.0f;
		wheel->suspension.frequency = front ? 1.8f : 2.0f;
	}

	def.engine.maxTorque = 380.0f;
	def.engine.peakTorqueRpm = 4500.0f;
	def.engine.idleTorqueScale = 0.6f;
	def.engine.maxRpmTorqueScale = 0.75f;
	def.engine.idleRpm = 900.0f;
	def.engine.maxRpm = 6800.0f;
	def.engine.inertia = 0.3f;
	def.engine.angularDamping = 0.08f;

	def.transmission.mode = b3_vehicleTransmissionAuto;
	float ratios[6] = { 3.3f, 2.1f, 1.5f, 1.15f, 0.92f, 0.78f };
	for ( int i = 0; i < 6; ++i )
	{
		def.transmission.gearRatios[i] = ratios[i];
	}
	def.transmission.gearCount = 6;
	def.transmission.reverseRatio = 3.2f;
	def.transmission.finalDrive = 3.9f;
	def.transmission.shiftUpRpm = 6000.0f;
	def.transmission.shiftDownRpm = 2600.0f;
	def.transmission.shiftTime = 0.15f;
	def.transmission.clutchReleaseTime = 0.2f;
	def.transmission.shiftLatency = 0.5f;
	def.transmission.clutchStrength = 10.0f;

	def.drivetrain.differentialCount = 1;
	def.drivetrain.differentials[0] = (b3VehicleDifferential){
		.leftWheel = 2,
		.rightWheel = 3,
		.torqueRatio = 1.0f,
		.leftRightSplit = 0.5f,
		.limitedSlipRatio = 1.4f,
	};
	for ( int i = 1; i < B3_MAX_VEHICLE_DIFFERENTIALS; ++i )
	{
		def.drivetrain.differentials[i] = (b3VehicleDifferential){ -1, -1, 0.0f, 0.5f, 1.0f };
	}
	def.drivetrain.centerLimitedSlipRatio = 1.4f;

	def.antiRoll.barCount = 2;
	def.antiRoll.bars[0] = (b3VehicleAntiRollBar){ 0, 1, 25000.0f };
	def.antiRoll.bars[1] = (b3VehicleAntiRollBar){ 2, 3, 15000.0f };
	for ( int i = 2; i < B3_MAX_VEHICLE_ANTI_ROLL_BARS; ++i )
	{
		def.antiRoll.bars[i] = (b3VehicleAntiRollBar){ -1, -1, 0.0f };
	}

	def.steering.maxAngle = b3DegToRad( 35.0f );
	def.steering.speedReference = 10.0f;
	def.steering.steerSpeed = 4.0f;
	def.steering.returnSpeed = 6.0f;
	def.steering.ackermann = 0.7f;

	def.assists.counterSteer = 0.2f;
	def.assists.driftGripScale = 1.0f;
	def.assists.tractionControl = true;
	def.assists.abs = false;
	def.assists.downforce = 1.5f;
	def.assists.drag = 0.6f;
	def.assists.rollInfluence = 0.25f;
	def.assists.airControl = 1.0f;
	def.assists.airAngularDamping = 2.0f;
	def.assists.selfRighting = 3.0f;
	def.assists.maxTiltAngle = 0.0f;
	def.assists.stickyTire = true;
	def.assists.stickySpeed = 0.25f;
	def.assists.stickyTime = 0.0f;

	def.internalValue = B3_SECRET_COOKIE;
	return def;
}

static float b3SanitizeFloat( float value, float lower, float upper, float fallback )
{
	if ( b3IsValidFloat( value ) == false )
	{
		return fallback;
	}

	return b3ClampFloat( value, lower, upper );
}

static b3VehicleChassis b3SanitizeChassis( b3VehicleChassis c )
{
	c.mass = b3SanitizeFloat( c.mass, 0.0f, FLT_MAX, 0.0f );
	if ( b3IsValidVec3( c.centerOfMassOffset ) == false )
	{
		c.centerOfMassOffset = b3Vec3_zero;
	}

	b3Vec3 up = b3IsValidVec3( c.up ) && b3LengthSquared( c.up ) > 1e-6f ? b3Normalize( c.up ) : b3Vec3_axisY;
	b3Vec3 forward = b3IsValidVec3( c.forward ) ? c.forward : b3Vec3_axisZ;
	forward = b3MulSub( forward, b3Dot( forward, up ), up );
	forward = b3LengthSquared( forward ) > 1e-6f ? b3Normalize( forward ) : b3Perp( up );
	c.up = up;
	c.forward = forward;
	c.maxSlopeAngle = b3SanitizeFloat( c.maxSlopeAngle, 0.0f, 0.5f * B3_PI, b3DegToRad( 80.0f ) );
	return c;
}

static b3VehicleWheelDef b3SanitizeWheel( b3VehicleWheelDef w )
{
	if ( b3IsValidVec3( w.position ) == false )
	{
		w.position = b3Vec3_zero;
	}

	w.radius = b3SanitizeFloat( w.radius, 0.01f, 100.0f, 0.34f );
	w.width = b3SanitizeFloat( w.width, 0.0f, 100.0f, 0.22f );
	w.inertia = b3SanitizeFloat( w.inertia, 1e-3f, FLT_MAX, 1.0f );
	w.steerFactor = b3SanitizeFloat( w.steerFactor, -10.0f, 10.0f, 0.0f );
	w.maxBrakeTorque = b3SanitizeFloat( w.maxBrakeTorque, 0.0f, FLT_MAX, 0.0f );
	w.maxHandBrakeTorque = b3SanitizeFloat( w.maxHandBrakeTorque, 0.0f, FLT_MAX, 0.0f );
	w.angularDamping = b3SanitizeFloat( w.angularDamping, 0.0f, FLT_MAX, 0.0f );
	if ( w.castType < b3_vehicleCastCylinder || w.castType > b3_vehicleCastRay )
	{
		w.castType = b3_vehicleCastCylinder;
	}

	b3VehicleSuspension* s = &w.suspension;
	s->maxLength = b3SanitizeFloat( s->maxLength, 0.0f, 100.0f, 0.4f );
	s->minLength = b3SanitizeFloat( s->minLength, 0.0f, s->maxLength, 0.0f );
	s->preload = b3SanitizeFloat( s->preload, -s->maxLength, 100.0f, 0.0f );
	s->frequency = b3SanitizeFloat( s->frequency, 0.0f, 100.0f, 0.0f );
	s->compressionDamping = b3SanitizeFloat( s->compressionDamping, 0.0f, 100.0f, 0.0f );
	s->reboundDamping = b3SanitizeFloat( s->reboundDamping, 0.0f, 100.0f, 0.0f );

	b3VehicleTire* t = &w.tire;
	t->longitudinalPeakSlip = b3SanitizeFloat( t->longitudinalPeakSlip, 1e-3f, 100.0f, 0.1f );
	t->longitudinalSlideSlip =
		b3SanitizeFloat( t->longitudinalSlideSlip, t->longitudinalPeakSlip, 100.0f, t->longitudinalPeakSlip );
	t->longitudinalPeakFriction = b3SanitizeFloat( t->longitudinalPeakFriction, 0.0f, 100.0f, 0.0f );
	t->longitudinalSlideFriction = b3SanitizeFloat( t->longitudinalSlideFriction, 0.0f, 100.0f, 0.0f );
	t->lateralPeakAngle = b3SanitizeFloat( t->lateralPeakAngle, 1e-3f, 0.5f * B3_PI, 0.12f );
	t->lateralSlideAngle = b3SanitizeFloat( t->lateralSlideAngle, t->lateralPeakAngle, 0.5f * B3_PI, t->lateralPeakAngle );
	t->lateralPeakFriction = b3SanitizeFloat( t->lateralPeakFriction, 0.0f, 100.0f, 0.0f );
	t->lateralSlideFriction = b3SanitizeFloat( t->lateralSlideFriction, 0.0f, 100.0f, 0.0f );
	t->rollingResistance = b3SanitizeFloat( t->rollingResistance, 0.0f, 10.0f, 0.0f );
	return w;
}

static b3VehicleEngine b3SanitizeEngine( b3VehicleEngine e )
{
	e.maxTorque = b3SanitizeFloat( e.maxTorque, 0.0f, FLT_MAX, 0.0f );
	e.idleRpm = b3SanitizeFloat( e.idleRpm, 0.0f, 1e6f, 900.0f );
	e.maxRpm = b3SanitizeFloat( e.maxRpm, e.idleRpm + 1.0f, 1e6f, e.idleRpm + 1.0f );
	e.peakTorqueRpm = b3SanitizeFloat( e.peakTorqueRpm, e.idleRpm, e.maxRpm, e.idleRpm );
	e.idleTorqueScale = b3SanitizeFloat( e.idleTorqueScale, 0.0f, 10.0f, 1.0f );
	e.maxRpmTorqueScale = b3SanitizeFloat( e.maxRpmTorqueScale, 0.0f, 10.0f, 1.0f );
	e.inertia = b3SanitizeFloat( e.inertia, 1e-3f, FLT_MAX, 0.3f );
	e.angularDamping = b3SanitizeFloat( e.angularDamping, 0.0f, FLT_MAX, 0.0f );
	return e;
}

static b3VehicleTransmission b3SanitizeTransmission( b3VehicleTransmission t )
{
	if ( t.mode != b3_vehicleTransmissionAuto && t.mode != b3_vehicleTransmissionManual )
	{
		t.mode = b3_vehicleTransmissionAuto;
	}

	t.gearCount = b3ClampInt( t.gearCount, 1, B3_MAX_VEHICLE_GEARS );
	for ( int i = 0; i < B3_MAX_VEHICLE_GEARS; ++i )
	{
		t.gearRatios[i] = b3SanitizeFloat( t.gearRatios[i], 0.0f, 1000.0f, 1.0f );
	}

	t.reverseRatio = b3SanitizeFloat( t.reverseRatio, 0.0f, 1000.0f, 3.0f );
	t.finalDrive = b3SanitizeFloat( t.finalDrive, 0.0f, 1000.0f, 1.0f );
	t.shiftUpRpm = b3SanitizeFloat( t.shiftUpRpm, 0.0f, 1e6f, 6000.0f );
	t.shiftDownRpm = b3SanitizeFloat( t.shiftDownRpm, 0.0f, t.shiftUpRpm, 0.0f );
	t.shiftTime = b3SanitizeFloat( t.shiftTime, 0.0f, 10.0f, 0.0f );
	t.clutchReleaseTime = b3SanitizeFloat( t.clutchReleaseTime, 0.0f, 10.0f, 0.0f );
	t.shiftLatency = b3SanitizeFloat( t.shiftLatency, 0.0f, 10.0f, 0.0f );
	t.clutchStrength = b3SanitizeFloat( t.clutchStrength, 0.0f, FLT_MAX, 0.0f );
	return t;
}

static int b3SanitizeWheelIndex( int index, int wheelCount )
{
	return 0 <= index && index < wheelCount ? index : -1;
}

static b3VehicleDrivetrain b3SanitizeDrivetrain( b3VehicleDrivetrain d, int wheelCount )
{
	d.differentialCount = b3ClampInt( d.differentialCount, 0, B3_MAX_VEHICLE_DIFFERENTIALS );
	for ( int i = 0; i < B3_MAX_VEHICLE_DIFFERENTIALS; ++i )
	{
		b3VehicleDifferential* diff = d.differentials + i;
		diff->leftWheel = b3SanitizeWheelIndex( diff->leftWheel, wheelCount );
		diff->rightWheel = b3SanitizeWheelIndex( diff->rightWheel, wheelCount );
		diff->torqueRatio = b3SanitizeFloat( diff->torqueRatio, 0.0f, 1.0f, 0.0f );
		diff->leftRightSplit = b3SanitizeFloat( diff->leftRightSplit, 0.0f, 1.0f, 0.5f );
		diff->limitedSlipRatio = b3SanitizeFloat( diff->limitedSlipRatio, 0.0f, FLT_MAX, 1.0f );
	}

	d.centerLimitedSlipRatio = b3SanitizeFloat( d.centerLimitedSlipRatio, 0.0f, FLT_MAX, 1.0f );
	return d;
}

static b3VehicleAntiRoll b3SanitizeAntiRoll( b3VehicleAntiRoll a, int wheelCount )
{
	a.barCount = b3ClampInt( a.barCount, 0, B3_MAX_VEHICLE_ANTI_ROLL_BARS );
	for ( int i = 0; i < B3_MAX_VEHICLE_ANTI_ROLL_BARS; ++i )
	{
		b3VehicleAntiRollBar* bar = a.bars + i;
		bar->leftWheel = b3SanitizeWheelIndex( bar->leftWheel, wheelCount );
		bar->rightWheel = b3SanitizeWheelIndex( bar->rightWheel, wheelCount );
		bar->stiffness = b3SanitizeFloat( bar->stiffness, 0.0f, FLT_MAX, 0.0f );
	}
	return a;
}

static b3VehicleSteering b3SanitizeSteering( b3VehicleSteering s )
{
	s.maxAngle = b3SanitizeFloat( s.maxAngle, 0.0f, 0.5f * B3_PI, 0.0f );
	s.speedReference = b3SanitizeFloat( s.speedReference, 0.0f, FLT_MAX, 0.0f );
	s.steerSpeed = b3SanitizeFloat( s.steerSpeed, 0.0f, FLT_MAX, 0.0f );
	s.returnSpeed = b3SanitizeFloat( s.returnSpeed, 0.0f, FLT_MAX, 0.0f );
	s.ackermann = b3SanitizeFloat( s.ackermann, 0.0f, 1.0f, 0.0f );
	return s;
}

static b3VehicleAssists b3SanitizeAssists( b3VehicleAssists a )
{
	a.counterSteer = b3SanitizeFloat( a.counterSteer, 0.0f, 10.0f, 0.0f );
	a.driftGripScale = b3SanitizeFloat( a.driftGripScale, 0.0f, 10.0f, 1.0f );
	a.downforce = b3SanitizeFloat( a.downforce, 0.0f, FLT_MAX, 0.0f );
	a.drag = b3SanitizeFloat( a.drag, 0.0f, FLT_MAX, 0.0f );
	a.rollInfluence = b3SanitizeFloat( a.rollInfluence, 0.0f, 1.0f, 0.0f );
	a.airControl = b3SanitizeFloat( a.airControl, 0.0f, FLT_MAX, 0.0f );
	a.airAngularDamping = b3SanitizeFloat( a.airAngularDamping, 0.0f, FLT_MAX, 0.0f );
	a.selfRighting = b3SanitizeFloat( a.selfRighting, 0.0f, FLT_MAX, 0.0f );
	a.maxTiltAngle = b3SanitizeFloat( a.maxTiltAngle, 0.0f, B3_PI, 0.0f );
	a.stickySpeed = b3SanitizeFloat( a.stickySpeed, 0.0f, FLT_MAX, 0.0f );
	a.stickyTime = b3SanitizeFloat( a.stickyTime, 0.0f, FLT_MAX, 0.0f );
	return a;
}

static b3Vehicle* b3GetVehicleFullId( b3World* world, b3VehicleId vehicleId )
{
	int id = vehicleId.index1 - 1;
	b3Vehicle* vehicle = b3Array_Get( world->vehicles, id );
	B3_ASSERT( vehicle->id == id && vehicle->generation == vehicleId.generation );
	return vehicle;
}

static b3Vehicle* b3GetVehicle( b3VehicleId vehicleId )
{
	b3World* world = b3GetWorld( vehicleId.world0 );
	return b3GetVehicleFullId( world, vehicleId );
}

static b3World* b3GetVehicleWorld( b3VehicleId vehicleId )
{
	return b3GetWorld( vehicleId.world0 );
}

static bool b3IsJointIdValidInternal( b3World* world, b3JointId jointId )
{
	if ( jointId.index1 < 1 || world->joints.count < jointId.index1 || jointId.world0 != world->worldId )
	{
		return false;
	}

	b3Joint* joint = world->joints.data + ( jointId.index1 - 1 );
	return joint->jointId == jointId.index1 - 1 && joint->generation == jointId.generation &&
		   joint->type == b3_vehicleWheelJoint;
}

static bool b3IsBodyIdValidInternal( b3World* world, b3BodyId bodyId )
{
	if ( bodyId.index1 < 1 || world->bodies.count < bodyId.index1 || bodyId.world0 != world->worldId )
	{
		return false;
	}

	b3Body* body = world->bodies.data + ( bodyId.index1 - 1 );
	return body->id == bodyId.index1 - 1 && body->generation == bodyId.generation;
}

static b3BodyId b3GetVehicleAnchor( b3World* world )
{
	if ( b3IsBodyIdValidInternal( world, world->vehicleAnchorId ) )
	{
		return world->vehicleAnchorId;
	}

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_staticBody;
	bodyDef.name = "vehicle anchor";
	b3WorldId worldId = { (uint16_t)( world->worldId + 1 ), world->generation };
	world->vehicleAnchorId = b3CreateBody( worldId, &bodyDef );
	return world->vehicleAnchorId;
}

static float b3GravityLength( b3World* world )
{
	float g = b3Length( world->gravity );
	return g > 0.0f ? g : 9.81f;
}

static float b3RestLength( const b3VehicleWheelDef* wheel, float gravity )
{
	const b3VehicleSuspension* s = &wheel->suspension;
	float omega = 2.0f * B3_PI * s->frequency;
	float compression = omega > 0.0f ? gravity / ( omega * omega ) : s->maxLength - s->minLength;
	return b3ClampFloat( s->maxLength + s->preload - compression, s->minLength, s->maxLength );
}

// Sprung masses from the center of mass and the wheel layout. Solves for the masses that
// balance the chassis weight and moments with the smallest variation (PhysX style).
static void b3ComputeSprungMasses( b3World* world, b3Vehicle* v )
{
	int n = v->wheelCount;
	float mass = b3Body_GetMass( v->chassisId );
	b3Vec3 center = b3Body_GetLocalCenter( v->chassisId );
	b3Vec3 up = v->chassis.up;
	b3Vec3 forward = v->chassis.forward;
	b3Vec3 left = b3Cross( up, forward );

	float x[B3_MAX_VEHICLE_WHEELS], z[B3_MAX_VEHICLE_WHEELS];
	float sx = 0.0f, sz = 0.0f, sxx = 0.0f, sxz = 0.0f, szz = 0.0f;
	for ( int i = 0; i < n; ++i )
	{
		b3Vec3 d = b3Sub( v->wheels[i].def.position, center );
		x[i] = b3Dot( d, left );
		z[i] = b3Dot( d, forward );
		sx += x[i];
		sz += z[i];
		sxx += x[i] * x[i];
		sxz += x[i] * z[i];
		szz += z[i] * z[i];
	}

	// [n sx sz; sx sxx sxz; sz sxz szz] * [a b c] = [mass 0 0]
	float c00 = sxx * szz - sxz * sxz;
	float c01 = sxz * sz - sx * szz;
	float c02 = sx * sxz - sxx * sz;
	float det = n * c00 + sx * c01 + sz * c02;

	float sprung[B3_MAX_VEHICLE_WHEELS];
	float scale = 1.0f + sxx + szz;
	if ( n >= 3 && b3AbsFloat( det ) > 1e-6f * scale * scale * scale )
	{
		float a = mass * c00 / det;
		float b = mass * c01 / det;
		float c = mass * c02 / det;
		for ( int i = 0; i < n; ++i )
		{
			sprung[i] = a + b * x[i] + c * z[i];
		}
	}
	else
	{
		for ( int i = 0; i < n; ++i )
		{
			sprung[i] = mass / n;
		}
	}

	float minMass = 0.05f * mass / n;
	float total = 0.0f;
	for ( int i = 0; i < n; ++i )
	{
		sprung[i] = b3MaxFloat( sprung[i], minMass );
		total += sprung[i];
	}

	B3_UNUSED( world );
	for ( int i = 0; i < n; ++i )
	{
		b3VehicleWheel* wheel = v->wheels + i;
		const b3VehicleSuspension* s = &wheel->def.suspension;
		float m = total > 0.0f ? sprung[i] * mass / total : 0.0f;
		float omega = 2.0f * B3_PI * s->frequency;
		wheel->sprungMass = m;
		wheel->stiffness = m * omega * omega;
		wheel->compressionDamping = 2.0f * m * s->compressionDamping * omega;
		wheel->reboundDamping = 2.0f * m * s->reboundDamping * omega;
	}
}

static void b3ApplyVehicleMass( b3World* world, b3Vehicle* v )
{
	b3MassData massData;
	massData.mass = v->chassis.mass > 0.0f ? v->chassis.mass : v->baseMass;
	float scale = v->baseMass > 0.0f ? massData.mass / v->baseMass : 1.0f;
	massData.inertia = b3MulSM( scale, v->baseInertia );
	massData.center = v->baseCenter;

	if ( v->chassis.autoCenterOfMass && v->wheelCount > 0 )
	{
		float g = b3GravityLength( world );
		b3Vec3 up = v->chassis.up;
		float height = 0.0f;
		for ( int i = 0; i < v->wheelCount; ++i )
		{
			const b3VehicleWheelDef* wheel = &v->wheels[i].def;
			height += b3Dot( wheel->position, up ) - b3RestLength( wheel, g );
		}
		height /= v->wheelCount;
		massData.center = b3MulAdd( massData.center, height - b3Dot( massData.center, up ), up );
	}

	massData.center = b3Add( massData.center, v->chassis.centerOfMassOffset );

	if ( massData.mass > 0.0f )
	{
		b3Body_SetMassData( v->chassisId, massData );
	}

	b3ComputeSprungMasses( world, v );
	v->massDirty = false;
}

b3VehicleId b3CreateVehicle( b3WorldId worldId, const b3VehicleDef* def )
{
	B3_CHECK_DEF( def );

	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == NULL )
	{
		return b3_nullVehicleId;
	}

	// Vehicles run outside the recorded op stream
	B3_ASSERT( world->recording == NULL );
	if ( world->recording != NULL )
	{
		return b3_nullVehicleId;
	}

	B3_ASSERT( b3Body_IsValid( def->chassisId ) );
	B3_ASSERT( b3Body_GetType( def->chassisId ) == b3_dynamicBody );
	B3_ASSERT( 0 < def->wheelCount && def->wheelCount <= B3_MAX_VEHICLE_WHEELS );
	if ( b3Body_IsValid( def->chassisId ) == false || def->wheelCount <= 0 )
	{
		return b3_nullVehicleId;
	}

	b3BodyId anchorId = b3GetVehicleAnchor( world );

	int id = b3AllocId( &world->vehicleIdPool );
	if ( id == world->vehicles.count )
	{
		b3Array_Push( world->vehicles, (b3Vehicle){ 0 } );
	}

	b3Vehicle* v = b3Array_Get( world->vehicles, id );
	uint16_t generation = v->generation;
	*v = (b3Vehicle){ 0 };
	v->id = id;
	v->generation = generation + 1;
	v->chassisId = def->chassisId;
	v->userData = def->userData;

	v->wheelCount = b3MinInt( def->wheelCount, B3_MAX_VEHICLE_WHEELS );
	v->chassis = b3SanitizeChassis( def->chassis );
	for ( int i = 0; i < v->wheelCount; ++i )
	{
		b3VehicleWheel* wheel = v->wheels + i;
		wheel->def = b3SanitizeWheel( def->wheels[i] );
		wheel->suspensionLength = wheel->def.suspension.maxLength;
		wheel->groundBodyId = anchorId;
		wheel->jointId = b3CreateVehicleWheelJointInternal( world, v->chassisId, anchorId );
	}

	v->engine = b3SanitizeEngine( def->engine );
	v->transmission = b3SanitizeTransmission( def->transmission );
	v->drivetrain = b3SanitizeDrivetrain( def->drivetrain, v->wheelCount );
	v->antiRoll = b3SanitizeAntiRoll( def->antiRoll, v->wheelCount );
	v->steering = b3SanitizeSteering( def->steering );
	v->assists = b3SanitizeAssists( def->assists );

	b3MassData massData = b3Body_GetMassData( v->chassisId );
	v->baseMass = massData.mass;
	v->baseCenter = massData.center;
	v->baseInertia = massData.inertia;

	v->engineSpeed = v->engine.idleRpm * B3_RPM_TO_RADIANS;
	v->gear = 1;

	b3ApplyVehicleMass( world, v );

	return (b3VehicleId){ id + 1, world->worldId, v->generation };
}

static void b3DestroyVehicleInternal( b3World* world, b3Vehicle* v )
{
	for ( int i = 0; i < v->wheelCount; ++i )
	{
		b3JointId jointId = v->wheels[i].jointId;
		if ( b3IsJointIdValidInternal( world, jointId ) )
		{
			b3Joint* joint = b3GetJointFullId( world, jointId );
			b3DestroyJointInternal( world, joint, false );
		}
	}

	b3FreeId( &world->vehicleIdPool, v->id );
	v->id = B3_NULL_INDEX;
}

void b3DestroyVehicle( b3VehicleId vehicleId )
{
	b3World* world = b3GetUnlockedWorld( vehicleId.world0 );
	if ( world == NULL )
	{
		return;
	}

	b3Vehicle* v = b3GetVehicleFullId( world, vehicleId );
	b3DestroyVehicleInternal( world, v );
}

void b3DestroyVehiclesOnBody( b3World* world, b3BodyId bodyId )
{
	if ( B3_ID_EQUALS( bodyId, world->vehicleAnchorId ) )
	{
		world->vehicleAnchorId = b3_nullBodyId;
	}

	for ( int i = 0; i < world->vehicles.count; ++i )
	{
		b3Vehicle* v = world->vehicles.data + i;
		if ( v->id != B3_NULL_INDEX && v->generation != 0 && B3_ID_EQUALS( v->chassisId, bodyId ) )
		{
			b3DestroyVehicleInternal( world, v );
		}
	}
}

bool b3Vehicle_IsValid( b3VehicleId id )
{
	if ( B3_MAX_WORLDS <= id.world0 )
	{
		return false;
	}

	b3World* world = b3GetWorld( id.world0 );
	if ( world == NULL || world->inUse == false || world->worldId != id.world0 )
	{
		return false;
	}

	if ( id.index1 < 1 || world->vehicles.count < id.index1 )
	{
		return false;
	}

	b3Vehicle* v = world->vehicles.data + ( id.index1 - 1 );
	return v->id == id.index1 - 1 && v->generation == id.generation;
}

int b3World_GetVehicleCount( b3WorldId worldId )
{
	b3World* world = b3GetWorldFromId( worldId );
	return b3GetIdCount( &world->vehicleIdPool );
}

b3BodyId b3Vehicle_GetChassisBody( b3VehicleId vehicleId )
{
	return b3GetVehicle( vehicleId )->chassisId;
}

int b3Vehicle_GetWheelCount( b3VehicleId vehicleId )
{
	return b3GetVehicle( vehicleId )->wheelCount;
}

static void b3WakeVehicle( b3Vehicle* v )
{
	b3Body_SetAwake( v->chassisId, true );
}

void b3Vehicle_SetInput( b3VehicleId vehicleId, b3VehicleInput input )
{
	b3Vehicle* v = b3GetVehicle( vehicleId );
	input.forward = b3SanitizeFloat( input.forward, -1.0f, 1.0f, 0.0f );
	input.brake = b3SanitizeFloat( input.brake, 0.0f, 1.0f, 0.0f );
	input.steer = b3SanitizeFloat( input.steer, -1.0f, 1.0f, 0.0f );
	input.handbrake = b3SanitizeFloat( input.handbrake, 0.0f, 1.0f, 0.0f );
	v->input = input;

	if ( input.forward != 0.0f || input.brake != 0.0f || input.steer != 0.0f || input.handbrake != 0.0f )
	{
		b3World* world = b3GetVehicleWorld( vehicleId );
		if ( world->locked == false )
		{
			b3WakeVehicle( v );
		}
	}
}

b3VehicleInput b3Vehicle_GetInput( b3VehicleId vehicleId )
{
	return b3GetVehicle( vehicleId )->input;
}

// Fully extended with no ground, so the readouts never report a stale contact
static void b3ClearWheelContact( b3VehicleWheel* wheel )
{
	wheel->hasContact = false;
	wheel->suspensionLength = wheel->def.suspension.maxLength;
	wheel->contactPoint = b3Pos_zero;
	wheel->contactNormal = b3Vec3_zero;
	wheel->groundShapeId = b3_nullShapeId;
	wheel->userMaterialId = 0;
}

static void b3ShiftTo( b3Vehicle* v, int gear )
{
	if ( v->gear == gear )
	{
		return;
	}

	v->gear = gear;
	v->shiftTimer = v->transmission.shiftTime;
	v->clutchTimer = v->shiftTimer > 0.0f ? 0.0f : v->transmission.clutchReleaseTime;
	v->latencyTimer = v->transmission.shiftLatency;
}

void b3Vehicle_SetGear( b3VehicleId vehicleId, int gear )
{
	b3Vehicle* v = b3GetVehicle( vehicleId );
	b3ShiftTo( v, b3ClampInt( gear, -1, v->transmission.gearCount ) );
}

void b3Vehicle_Reset( b3VehicleId vehicleId, b3Pos position, b3Quat rotation )
{
	b3World* world = b3GetUnlockedWorld( vehicleId.world0 );
	if ( world == NULL )
	{
		return;
	}

	b3Vehicle* v = b3GetVehicleFullId( world, vehicleId );
	b3Body_SetTransform( v->chassisId, position, rotation );
	b3Body_SetLinearVelocity( v->chassisId, b3Vec3_zero );
	b3Body_SetAngularVelocity( v->chassisId, b3Vec3_zero );
	b3WakeVehicle( v );

	for ( int i = 0; i < v->wheelCount; ++i )
	{
		b3VehicleWheel* wheel = v->wheels + i;
		wheel->omega = 0.0f;
		wheel->stickyTimer = 0.0f;
		b3ClearWheelContact( wheel );
		if ( b3IsJointIdValidInternal( world, wheel->jointId ) )
		{
			b3Joint* joint = b3GetJointFullId( world, wheel->jointId );
			b3VehicleWheelJoint* sim = &b3GetJointSim( world, joint )->vehicleWheelJoint;
			sim->omega = 0.0f;
			sim->springImpulse = 0.0f;
			sim->bumpImpulse = 0.0f;
			sim->longitudinalImpulse = 0.0f;
			sim->lateralImpulse = 0.0f;
			sim->hasContact = false;
		}
	}

	v->engineSpeed = v->engine.idleRpm * B3_RPM_TO_RADIANS;
	v->gear = 1;
	v->shiftTimer = 0.0f;
	v->clutchTimer = 0.0f;
	v->latencyTimer = 0.0f;
	v->steer = 0.0f;
	v->stuckTimer = 0.0f;
	v->stuck = false;
}

void b3Vehicle_SetChassis( b3VehicleId vehicleId, b3VehicleChassis chassis )
{
	b3Vehicle* v = b3GetVehicle( vehicleId );
	v->chassis = b3SanitizeChassis( chassis );
	v->massDirty = true;
	b3WakeVehicle( v );
}

b3VehicleChassis b3Vehicle_GetChassis( b3VehicleId vehicleId )
{
	return b3GetVehicle( vehicleId )->chassis;
}

void b3Vehicle_SetWheel( b3VehicleId vehicleId, int wheelIndex, b3VehicleWheelDef wheel )
{
	b3Vehicle* v = b3GetVehicle( vehicleId );
	B3_ASSERT( 0 <= wheelIndex && wheelIndex < v->wheelCount );
	if ( wheelIndex < 0 || v->wheelCount <= wheelIndex )
	{
		return;
	}

	v->wheels[wheelIndex].def = b3SanitizeWheel( wheel );
	v->massDirty = true;
	b3WakeVehicle( v );
}

b3VehicleWheelDef b3Vehicle_GetWheel( b3VehicleId vehicleId, int wheelIndex )
{
	b3Vehicle* v = b3GetVehicle( vehicleId );
	B3_ASSERT( 0 <= wheelIndex && wheelIndex < v->wheelCount );
	return v->wheels[b3ClampInt( wheelIndex, 0, v->wheelCount - 1 )].def;
}

void b3Vehicle_SetEngine( b3VehicleId vehicleId, b3VehicleEngine engine )
{
	b3Vehicle* v = b3GetVehicle( vehicleId );
	v->engine = b3SanitizeEngine( engine );
	b3WakeVehicle( v );
}

b3VehicleEngine b3Vehicle_GetEngine( b3VehicleId vehicleId )
{
	return b3GetVehicle( vehicleId )->engine;
}

void b3Vehicle_SetTransmission( b3VehicleId vehicleId, b3VehicleTransmission transmission )
{
	b3Vehicle* v = b3GetVehicle( vehicleId );
	v->transmission = b3SanitizeTransmission( transmission );
	v->gear = b3ClampInt( v->gear, -1, v->transmission.gearCount );
	b3WakeVehicle( v );
}

b3VehicleTransmission b3Vehicle_GetTransmission( b3VehicleId vehicleId )
{
	return b3GetVehicle( vehicleId )->transmission;
}

void b3Vehicle_SetDrivetrain( b3VehicleId vehicleId, b3VehicleDrivetrain drivetrain )
{
	b3Vehicle* v = b3GetVehicle( vehicleId );
	v->drivetrain = b3SanitizeDrivetrain( drivetrain, v->wheelCount );
	b3WakeVehicle( v );
}

b3VehicleDrivetrain b3Vehicle_GetDrivetrain( b3VehicleId vehicleId )
{
	return b3GetVehicle( vehicleId )->drivetrain;
}

void b3Vehicle_SetAntiRoll( b3VehicleId vehicleId, b3VehicleAntiRoll antiRoll )
{
	b3Vehicle* v = b3GetVehicle( vehicleId );
	v->antiRoll = b3SanitizeAntiRoll( antiRoll, v->wheelCount );
	b3WakeVehicle( v );
}

b3VehicleAntiRoll b3Vehicle_GetAntiRoll( b3VehicleId vehicleId )
{
	return b3GetVehicle( vehicleId )->antiRoll;
}

void b3Vehicle_SetSteering( b3VehicleId vehicleId, b3VehicleSteering steering )
{
	b3Vehicle* v = b3GetVehicle( vehicleId );
	v->steering = b3SanitizeSteering( steering );
	b3WakeVehicle( v );
}

b3VehicleSteering b3Vehicle_GetSteering( b3VehicleId vehicleId )
{
	return b3GetVehicle( vehicleId )->steering;
}

void b3Vehicle_SetAssists( b3VehicleId vehicleId, b3VehicleAssists assists )
{
	b3Vehicle* v = b3GetVehicle( vehicleId );
	v->assists = b3SanitizeAssists( assists );
	b3WakeVehicle( v );
}

b3VehicleAssists b3Vehicle_GetAssists( b3VehicleId vehicleId )
{
	return b3GetVehicle( vehicleId )->assists;
}

void b3Vehicle_SetUserData( b3VehicleId vehicleId, void* userData )
{
	b3GetVehicle( vehicleId )->userData = userData;
}

void* b3Vehicle_GetUserData( b3VehicleId vehicleId )
{
	return b3GetVehicle( vehicleId )->userData;
}

b3VehicleState b3Vehicle_GetState( b3VehicleId vehicleId )
{
	b3Vehicle* v = b3GetVehicle( vehicleId );
	b3VehicleState state = { 0 };
	state.speed = v->speed;
	state.engineRpm = v->engineSpeed * B3_RADIANS_TO_RPM;
	state.gear = v->gear;
	state.isShifting = v->shiftTimer > 0.0f;
	state.throttle = v->throttle;
	state.brake = v->brake;
	state.steerAngle = v->steer;
	state.wheelsInContact = v->wheelsInContact;
	state.stuck = v->stuck;
	return state;
}

static float b3GetWheelCompression( const b3VehicleWheel* wheel )
{
	const b3VehicleSuspension* s = &wheel->def.suspension;
	float travel = s->maxLength - s->minLength;
	return travel > 0.0f ? b3ClampFloat( ( s->maxLength - wheel->suspensionLength ) / travel, 0.0f, 1.0f ) : 0.0f;
}

static b3WorldTransform b3GetWheelTransform( const b3Vehicle* v, const b3VehicleWheel* wheel, b3WorldTransform chassis )
{
	b3Vec3 up = v->chassis.up;
	b3Vec3 axle = b3Cross( up, v->chassis.forward );
	b3Vec3 localCenter = b3MulSub( wheel->def.position, wheel->suspensionLength, up );
	b3Quat steer = b3MakeQuatFromAxisAngle( up, wheel->steerAngle );
	b3Quat spin = b3MakeQuatFromAxisAngle( axle, wheel->rotationAngle );
	b3Quat localRotation = b3MulQuat( steer, spin );

	b3WorldTransform result;
	result.p = b3TransformWorldPoint( chassis, localCenter );
	result.q = b3MulQuat( chassis.q, localRotation );
	return result;
}

// The one place the wheel readout is built, b3Vehicle_GetWheelData packs this
static b3VehicleWheelState b3MakeWheelState( const b3VehicleWheel* wheel )
{
	b3VehicleWheelState state = { 0 };
	state.hasContact = wheel->hasContact;
	state.contactPoint = wheel->contactPoint;
	state.contactNormal = wheel->contactNormal;
	state.groundShapeId = wheel->groundShapeId;
	state.userMaterialId = wheel->userMaterialId;
	state.suspensionLength = wheel->suspensionLength;
	state.compression = b3GetWheelCompression( wheel );
	state.load = wheel->load;
	state.longitudinalForce = wheel->longitudinalForce;
	state.lateralForce = wheel->lateralForce;
	state.slipRatio = wheel->slipRatio;
	state.slipAngle = wheel->slipAngle;
	state.angularVelocity = wheel->omega;
	state.rotationAngle = wheel->rotationAngle;
	state.steerAngle = wheel->steerAngle;
	state.driveTorque = wheel->driveTorque;
	state.brakeTorque = wheel->brakeTorque;
	state.isSticky = wheel->isSticky;
	return state;
}

b3VehicleWheelState b3Vehicle_GetWheelState( b3VehicleId vehicleId, int wheelIndex )
{
	b3Vehicle* v = b3GetVehicle( vehicleId );
	B3_ASSERT( 0 <= wheelIndex && wheelIndex < v->wheelCount );
	return b3MakeWheelState( v->wheels + b3ClampInt( wheelIndex, 0, v->wheelCount - 1 ) );
}

b3WorldTransform b3Vehicle_GetWheelTransform( b3VehicleId vehicleId, int wheelIndex )
{
	b3Vehicle* v = b3GetVehicle( vehicleId );
	B3_ASSERT( 0 <= wheelIndex && wheelIndex < v->wheelCount );
	b3VehicleWheel* wheel = v->wheels + b3ClampInt( wheelIndex, 0, v->wheelCount - 1 );
	return b3GetWheelTransform( v, wheel, b3Body_GetTransform( v->chassisId ) );
}

// Every documented word offset, JavaScript readers hard code them
#define B3_CHECK_WHEEL_WORD( field, word )                                                                                       \
	_Static_assert( offsetof( b3VehicleWheelData, field ) == ( word ) * 4, "wheel data layout is part of the API" )
B3_CHECK_WHEEL_WORD( position, 0 );
B3_CHECK_WHEEL_WORD( rotation, 3 );
B3_CHECK_WHEEL_WORD( contactPoint, 7 );
B3_CHECK_WHEEL_WORD( contactNormal, 10 );
B3_CHECK_WHEEL_WORD( suspensionLength, 13 );
B3_CHECK_WHEEL_WORD( compression, 14 );
B3_CHECK_WHEEL_WORD( load, 15 );
B3_CHECK_WHEEL_WORD( longitudinalForce, 16 );
B3_CHECK_WHEEL_WORD( lateralForce, 17 );
B3_CHECK_WHEEL_WORD( slipRatio, 18 );
B3_CHECK_WHEEL_WORD( slipAngle, 19 );
B3_CHECK_WHEEL_WORD( angularVelocity, 20 );
B3_CHECK_WHEEL_WORD( rotationAngle, 21 );
B3_CHECK_WHEEL_WORD( steerAngle, 22 );
B3_CHECK_WHEEL_WORD( driveTorque, 23 );
B3_CHECK_WHEEL_WORD( brakeTorque, 24 );
B3_CHECK_WHEEL_WORD( flags, 25 );
B3_CHECK_WHEEL_WORD( groundShapeId, 26 );
B3_CHECK_WHEEL_WORD( userMaterialId, 28 );
_Static_assert( sizeof( b3VehicleWheelData ) == 30 * 4, "wheel data layout is part of the API" );
#undef B3_CHECK_WHEEL_WORD

int b3Vehicle_GetWheelData( b3VehicleId vehicleId, b3VehicleWheelData* data, int capacity )
{
	b3Vehicle* v = b3GetVehicle( vehicleId );
	int count = b3ClampInt( capacity, 0, v->wheelCount );
	b3WorldTransform chassis = b3Body_GetTransform( v->chassisId );

	for ( int i = 0; i < count; ++i )
	{
		const b3VehicleWheel* wheel = v->wheels + i;
		b3VehicleWheelState state = b3MakeWheelState( wheel );
		b3WorldTransform xf = b3GetWheelTransform( v, wheel, chassis );

		b3VehicleWheelData* out = data + i;
		out->position = (b3Vec3){ (float)xf.p.x, (float)xf.p.y, (float)xf.p.z };
		out->rotation = xf.q;
		out->contactPoint = (b3Vec3){ (float)state.contactPoint.x, (float)state.contactPoint.y, (float)state.contactPoint.z };
		out->contactNormal = state.contactNormal;
		out->suspensionLength = state.suspensionLength;
		out->compression = state.compression;
		out->load = state.load;
		out->longitudinalForce = state.longitudinalForce;
		out->lateralForce = state.lateralForce;
		out->slipRatio = state.slipRatio;
		out->slipAngle = state.slipAngle;
		out->angularVelocity = state.angularVelocity;
		out->rotationAngle = state.rotationAngle;
		out->steerAngle = state.steerAngle;
		out->driveTorque = state.driveTorque;
		out->brakeTorque = state.brakeTorque;
		out->flags = ( state.hasContact ? b3_vehicleWheelContact : 0u ) | ( state.isSticky ? b3_vehicleWheelSticky : 0u );
		out->groundShapeId = state.groundShapeId;
		out->userMaterialId = state.userMaterialId;
	}

	return count;
}

float b3Vehicle_GetSprungMass( b3VehicleId vehicleId, int wheelIndex )
{
	b3Vehicle* v = b3GetVehicle( vehicleId );
	B3_ASSERT( 0 <= wheelIndex && wheelIndex < v->wheelCount );
	return v->wheels[b3ClampInt( wheelIndex, 0, v->wheelCount - 1 )].sprungMass;
}

// Pre-step

typedef struct b3WheelCastContext
{
	b3World* world;
	int chassisBodyId;
	b3Vec3 up;
	float minSlopeCos;

	bool hit;
	float fraction;
	b3Pos point;
	b3Vec3 normal;
	b3ShapeId shapeId;
	uint64_t userMaterialId;
	int triangleIndex;
	int childIndex;

	// Closest hit the shape cast could not resolve, it may be in front of the accepted hit
	bool degenerate;
	float degenerateFraction;
} b3WheelCastContext;

static float b3WheelCastCallback( b3ShapeId shapeId, b3Pos point, b3Vec3 normal, float fraction, uint64_t userMaterialId,
								  int triangleIndex, int childIndex, void* context )
{
	b3WheelCastContext* ctx = context;
	b3Shape* shape = b3Array_Get( ctx->world->shapes, shapeId.index1 - 1 );
	if ( shape->bodyId == ctx->chassisBodyId || shape->sensorIndex != B3_NULL_INDEX )
	{
		return -1.0f;
	}

	if ( b3LengthSquared( normal ) < 0.25f )
	{
		if ( fraction > 0.0f )
		{
			// The shape cast did not converge, the caller falls back to a ray if nothing closer was hit
			if ( ctx->degenerate == false || fraction < ctx->degenerateFraction )
			{
				ctx->degenerate = true;
				ctx->degenerateFraction = fraction;
			}
			return -1.0f;
		}

		// Started inside, there is no normal
		normal = ctx->up;
	}

	if ( b3Dot( normal, ctx->up ) < ctx->minSlopeCos )
	{
		return -1.0f;
	}

	ctx->hit = true;
	ctx->fraction = fraction;
	ctx->point = point;
	ctx->normal = normal;
	ctx->shapeId = shapeId;
	ctx->userMaterialId = userMaterialId;
	ctx->triangleIndex = triangleIndex;
	ctx->childIndex = childIndex;
	return fraction;
}

static float b3TireCurve( float slip, float peakSlip, float peakFriction, float slideSlip, float slideFriction )
{
	if ( slip <= peakSlip )
	{
		return peakFriction * slip / peakSlip;
	}

	if ( slip < slideSlip )
	{
		float t = ( slip - peakSlip ) / ( slideSlip - peakSlip );
		return b3LerpFloat( peakFriction, slideFriction, t );
	}

	return slideFriction;
}

static float b3EngineTorqueAt( const b3VehicleEngine* e, float rpm )
{
	if ( rpm >= e->maxRpm )
	{
		return 0.0f;
	}

	float scale;
	if ( rpm <= e->peakTorqueRpm )
	{
		float range = e->peakTorqueRpm - e->idleRpm;
		float t = range > 0.0f ? b3ClampFloat( ( rpm - e->idleRpm ) / range, 0.0f, 1.0f ) : 1.0f;
		scale = b3LerpFloat( e->idleTorqueScale, 1.0f, t );
	}
	else
	{
		float range = e->maxRpm - e->peakTorqueRpm;
		float t = range > 0.0f ? b3ClampFloat( ( rpm - e->peakTorqueRpm ) / range, 0.0f, 1.0f ) : 1.0f;
		scale = b3LerpFloat( 1.0f, e->maxRpmTorqueScale, t );
	}

	return scale * e->maxTorque;
}

// Moves torque away from the faster side once the speed ratio passes the limited slip ratio
static void b3LimitSlip( float speedA, float speedB, float ratio, float* shareA, float* shareB )
{
	if ( ratio <= 1.0f )
	{
		return;
	}

	float a = b3AbsFloat( speedA );
	float b = b3AbsFloat( speedB );
	float total = *shareA + *shareB;
	const float minSpeed = 1.0f;
	if ( a > ratio * b3MaxFloat( b, minSpeed ) )
	{
		float excess = a / b3MaxFloat( b, minSpeed ) - ratio;
		float f = b3ClampFloat( excess / ( 0.5f * ratio ), 0.0f, 1.0f );
		*shareA *= 1.0f - f;
		*shareB = total - *shareA;
	}
	else if ( b > ratio * b3MaxFloat( a, minSpeed ) )
	{
		float excess = b / b3MaxFloat( a, minSpeed ) - ratio;
		float f = b3ClampFloat( excess / ( 0.5f * ratio ), 0.0f, 1.0f );
		*shareB *= 1.0f - f;
		*shareA = total - *shareB;
	}
}

// Torque share per wheel from the differentials. Shares sum to 1 over all driven wheels.
static void b3ComputeTorqueShares( const b3Vehicle* v, float shares[B3_MAX_VEHICLE_WHEELS] )
{
	for ( int i = 0; i < B3_MAX_VEHICLE_WHEELS; ++i )
	{
		shares[i] = 0.0f;
	}

	const b3VehicleDrivetrain* d = &v->drivetrain;
	float diffShares[B3_MAX_VEHICLE_DIFFERENTIALS];
	float diffSpeeds[B3_MAX_VEHICLE_DIFFERENTIALS];
	float totalShare = 0.0f;
	for ( int i = 0; i < d->differentialCount; ++i )
	{
		const b3VehicleDifferential* diff = d->differentials + i;
		diffShares[i] = diff->torqueRatio;
		float speed = 0.0f;
		int count = 0;
		if ( diff->leftWheel >= 0 )
		{
			speed += b3AbsFloat( v->wheels[diff->leftWheel].omega );
			count += 1;
		}
		if ( diff->rightWheel >= 0 )
		{
			speed += b3AbsFloat( v->wheels[diff->rightWheel].omega );
			count += 1;
		}
		diffSpeeds[i] = count > 0 ? speed / count : 0.0f;
		totalShare += diffShares[i];
	}

	if ( totalShare <= 0.0f )
	{
		return;
	}

	// Center limited slip: each differential against the slowest one
	if ( d->differentialCount > 1 && d->centerLimitedSlipRatio > 1.0f )
	{
		int slowest = 0;
		for ( int i = 1; i < d->differentialCount; ++i )
		{
			if ( diffShares[i] > 0.0f && ( diffShares[slowest] == 0.0f || diffSpeeds[i] < diffSpeeds[slowest] ) )
			{
				slowest = i;
			}
		}

		for ( int i = 0; i < d->differentialCount; ++i )
		{
			if ( i != slowest && diffShares[i] > 0.0f )
			{
				b3LimitSlip( diffSpeeds[i], diffSpeeds[slowest], d->centerLimitedSlipRatio, diffShares + i,
							 diffShares + slowest );
			}
		}
	}

	for ( int i = 0; i < d->differentialCount; ++i )
	{
		const b3VehicleDifferential* diff = d->differentials + i;
		float share = diffShares[i] / totalShare;
		if ( diff->leftWheel >= 0 && diff->rightWheel >= 0 && diff->leftWheel != diff->rightWheel )
		{
			float left = diff->leftRightSplit;
			float right = 1.0f - left;
			b3LimitSlip( v->wheels[diff->leftWheel].omega, v->wheels[diff->rightWheel].omega, diff->limitedSlipRatio,
						 &left, &right );
			shares[diff->leftWheel] += share * left;
			shares[diff->rightWheel] += share * right;
		}
		else if ( diff->leftWheel >= 0 )
		{
			shares[diff->leftWheel] += share;
		}
		else if ( diff->rightWheel >= 0 )
		{
			shares[diff->rightWheel] += share;
		}
	}
}

static float b3GetGearRatio( const b3Vehicle* v )
{
	const b3VehicleTransmission* t = &v->transmission;
	if ( v->gear > 0 )
	{
		return t->gearRatios[b3MinInt( v->gear, t->gearCount ) - 1] * t->finalDrive;
	}

	if ( v->gear < 0 )
	{
		return -t->reverseRatio * t->finalDrive;
	}

	return 0.0f;
}

static void b3UpdateGearbox( b3Vehicle* v, float dt, float shiftRpm, bool blockUpshift )
{
	const b3VehicleTransmission* t = &v->transmission;

	if ( v->shiftTimer > 0.0f )
	{
		v->shiftTimer -= dt;
		if ( v->shiftTimer <= 0.0f )
		{
			v->shiftTimer = 0.0f;
			v->clutchTimer = t->clutchReleaseTime;
		}
	}
	else if ( v->clutchTimer > 0.0f )
	{
		v->clutchTimer = b3MaxFloat( v->clutchTimer - dt, 0.0f );
	}

	v->latencyTimer = b3MaxFloat( v->latencyTimer - dt, 0.0f );

	float forward = v->input.forward;
	float brake = v->input.brake;
	float throttle = 0.0f;
	const float switchSpeed = 1.0f;

	if ( t->mode == b3_vehicleTransmissionAuto )
	{
		// Without forward input the brake doubles as reverse: it brakes while moving forward and
		// selects reverse and drives backward once below the switch speed
		if ( forward == 0.0f && brake > 0.0f )
		{
			forward = -brake;
			brake = 0.0f;
		}

		if ( forward > 0.0f )
		{
			if ( v->gear < 0 && v->speed < -switchSpeed )
			{
				brake = b3MaxFloat( brake, forward );
			}
			else
			{
				if ( v->gear <= 0 )
				{
					b3ShiftTo( v, 1 );
				}
				throttle = forward;
			}
		}
		else if ( forward < 0.0f )
		{
			if ( v->gear > 0 && v->speed > switchSpeed )
			{
				brake = b3MaxFloat( brake, -forward );
			}
			else
			{
				if ( v->gear >= 0 )
				{
					b3ShiftTo( v, -1 );
				}
				throttle = -forward;
			}
		}

		if ( v->gear > 0 && v->shiftTimer <= 0.0f && v->latencyTimer <= 0.0f )
		{
			float rpm = shiftRpm;
			if ( rpm > t->shiftUpRpm && v->gear < t->gearCount && blockUpshift == false && throttle > 0.0f )
			{
				b3ShiftTo( v, v->gear + 1 );
			}
			else if ( rpm < t->shiftDownRpm && v->gear > 1 )
			{
				float current = t->gearRatios[v->gear - 1];
				float lower = t->gearRatios[v->gear - 2];
				float predicted = current > 0.0f ? rpm * lower / current : rpm;
				if ( predicted < 0.9f * t->shiftUpRpm )
				{
					b3ShiftTo( v, v->gear - 1 );
				}
			}
		}
	}
	else
	{
		throttle = b3MaxFloat( forward, 0.0f );
		brake = b3MaxFloat( brake, -forward );
	}

	if ( v->shiftTimer > 0.0f )
	{
		throttle = 0.0f;
	}

	v->throttle = throttle;
	v->brake = brake;
}

typedef struct b3WheelFrame
{
	b3Vec3 forward;
	b3Vec3 lateral;
	b3Vec3 rollOffset;
	float cosAngle;
	float groundScale;
	b3BodyId groundBodyId;
} b3WheelFrame;

static void b3CastWheel( b3World* world, b3Vehicle* v, b3VehicleWheel* wheel, b3WorldTransform xf, b3Vec3 up,
						 b3Vec3 heading, b3Vec3 axle, b3BodyId anchorId, b3WheelFrame* frame )
{
	const b3VehicleWheelDef* def = &wheel->def;
	const b3VehicleSuspension* s = &def->suspension;
	float radius = def->radius;

	b3Vec3 down = b3Neg( up );
	b3Pos attach = b3TransformWorldPoint( xf, def->position );

	b3Vec3 points[2 * B3_VEHICLE_RING_COUNT];
	b3ShapeProxy proxy = { points, 1, 0.0f };
	points[0] = b3Vec3_zero;

	float startLength = s->minLength - 0.5f * radius;
	float endLength = s->maxLength;
	float contactOffset = 0.0f;

	if ( def->castType == b3_vehicleCastCylinder )
	{
		float rounding = b3MinFloat( 0.05f, 0.5f * b3MinFloat( radius, 0.5f * def->width ) );
		float core = radius - rounding;
		float halfWidth = b3MaxFloat( 0.5f * def->width - rounding, 0.0f );
		int sides = halfWidth > 0.0f ? 2 : 1;
		int count = 0;
		for ( int side = 0; side < sides; ++side )
		{
			b3Vec3 offset = b3MulSV( side == 0 ? halfWidth : -halfWidth, axle );
			for ( int k = 0; k < B3_VEHICLE_RING_COUNT; ++k )
			{
				b3Vec3 p = b3MulAdd( offset, core * b3_ringCos[k], up );
				points[count++] = b3MulAdd( p, core * b3_ringSin[k], heading );
			}
		}
		proxy.count = count;
		proxy.radius = rounding;
	}
	else if ( def->castType == b3_vehicleCastSphere )
	{
		proxy.radius = radius;
	}
	else
	{
		// The ray shares the shape cast's start and ends one radius below full extension, where the
		// tire bottom is
		endLength = s->maxLength + radius;
		contactOffset = radius;
	}

	float castLength = endLength - startLength;
	b3Pos origin = b3OffsetPos( attach, b3MulSV( startLength, down ) );

	b3WheelCastContext ctx = { 0 };
	ctx.world = world;
	ctx.chassisBodyId = v->chassisId.index1 - 1;
	ctx.up = up;
	ctx.minSlopeCos = b3ComputeCosSin( v->chassis.maxSlopeAngle ).cosine;

	b3WorldId worldId = { (uint16_t)( world->worldId + 1 ), world->generation };
	if ( castLength > 0.0f )
	{
		if ( def->castType == b3_vehicleCastRay )
		{
			b3World_CastRay( worldId, origin, b3MulSV( castLength, down ), v->chassis.castFilter, b3WheelCastCallback,
							 &ctx );
		}
		else
		{
			b3World_CastShape( worldId, origin, &proxy, b3MulSV( castLength, down ), v->chassis.castFilter,
							   b3WheelCastCallback, &ctx );
		}
	}

	// A degenerate hit in front of the accepted one could be a thin obstacle the wheel would sink
	// through, so retry with a ray. The ray is narrower than the wheel and can pass beside what the
	// shape cast hit, so keep the shape hit unless the ray gives a shorter suspension.
	if ( ctx.degenerate && ( ctx.hit == false || ctx.degenerateFraction < ctx.fraction ) )
	{
		b3WheelCastContext shapeHit = ctx;
		float shapeLength = startLength + shapeHit.fraction * castLength - contactOffset;
		float rayLength = s->maxLength + radius - startLength;
		ctx.hit = false;
		ctx.degenerate = false;
		b3World_CastRay( worldId, origin, b3MulSV( rayLength, down ), v->chassis.castFilter, b3WheelCastCallback, &ctx );
		float rayHitLength = startLength + ctx.fraction * rayLength - radius;
		if ( ctx.hit && ( shapeHit.hit == false || rayHitLength < shapeLength ) )
		{
			castLength = rayLength;
			contactOffset = radius;
		}
		else
		{
			ctx = shapeHit;
		}
	}

	frame->groundBodyId = anchorId;
	frame->groundScale = 1.0f;
	frame->cosAngle = 1.0f;

	if ( ctx.hit == false )
	{
		b3ClearWheelContact( wheel );
		return;
	}

	float length = startLength + ctx.fraction * castLength - contactOffset;
	wheel->hasContact = true;
	wheel->suspensionLength = b3MinFloat( length, s->maxLength );
	// Keep the contact on the wheel center plane, the cylinder can touch at either edge
	float sideOffset = b3Dot( b3SubPos( ctx.point, attach ), axle );
	wheel->contactPoint = b3OffsetPos( ctx.point, b3MulSV( -sideOffset, axle ) );
	wheel->contactNormal = ctx.normal;
	wheel->groundShapeId = ctx.shapeId;
	wheel->userMaterialId = ctx.userMaterialId;

	b3Shape* shape = b3Array_Get( world->shapes, ctx.shapeId.index1 - 1 );
	b3Body* groundBody = b3Array_Get( world->bodies, shape->bodyId );
	if ( groundBody->type != b3_staticBody )
	{
		frame->groundBodyId = (b3BodyId){ groundBody->id + 1, world->worldId, groundBody->generation };
	}

	// The material of the hit triangle or compound child, not just the shape's first material
	int materialIndex = b3GetShapeMaterialIndex( shape, ctx.childIndex, ctx.triangleIndex );
	frame->groundScale = b3GetShapeMaterials( shape )[materialIndex].friction / B3_VEHICLE_REFERENCE_FRICTION;
	frame->cosAngle = b3MaxFloat( b3Dot( ctx.normal, up ), 0.1f );
}

static void b3UpdateVehicle( b3World* world, b3Vehicle* v, float dt )
{
	if ( b3IsBodyIdValidInternal( world, v->chassisId ) == false )
	{
		return;
	}

	b3Body* body = b3GetBodyFullId( world, v->chassisId );
	if ( body->setIndex == b3_disabledSet || body->type != b3_dynamicBody )
	{
		return;
	}

	b3VehicleInput* input = &v->input;
	bool hasInput = input->forward != 0.0f || input->brake != 0.0f || input->steer != 0.0f || input->handbrake != 0.0f;
	if ( body->setIndex >= b3_firstSleepingSet )
	{
		if ( hasInput == false && v->massDirty == false )
		{
			return;
		}

		b3WakeBody( world, body );
	}

	if ( v->massDirty )
	{
		b3ApplyVehicleMass( world, v );
	}

	b3BodyId anchorId = b3GetVehicleAnchor( world );
	int wheelCount = v->wheelCount;
	float inv_h = world->inv_h;

	// Read back the solver results of the previous step
	for ( int i = 0; i < wheelCount; ++i )
	{
		b3VehicleWheel* wheel = v->wheels + i;
		wheel->load = 0.0f;
		wheel->longitudinalForce = 0.0f;
		wheel->lateralForce = 0.0f;

		if ( b3IsJointIdValidInternal( world, wheel->jointId ) == false )
		{
			wheel->jointId = b3_nullJointId;
			continue;
		}

		b3Joint* joint = b3GetJointFullId( world, wheel->jointId );
		b3VehicleWheelJoint* sim = &b3GetJointSim( world, joint )->vehicleWheelJoint;
		wheel->omega = b3IsValidFloat( sim->omega ) ? sim->omega : 0.0f;
		if ( sim->hasContact )
		{
			wheel->load = inv_h * ( sim->springImpulse + sim->bumpImpulse );
			wheel->longitudinalForce = inv_h * sim->longitudinalImpulse;
			wheel->lateralForce = inv_h * sim->lateralImpulse;
		}
	}

	b3WorldTransform xf = b3Body_GetTransform( v->chassisId );
	b3Pos centerOfMass = b3Body_GetWorldCenter( v->chassisId );
	b3Vec3 linearVelocity = b3Body_GetLinearVelocity( v->chassisId );
	b3Vec3 angularVelocity = b3Body_GetAngularVelocity( v->chassisId );

	b3Vec3 forward = b3RotateVector( xf.q, v->chassis.forward );
	b3Vec3 up = b3RotateVector( xf.q, v->chassis.up );
	b3Vec3 left = b3Cross( up, forward );

	float gravityLength = b3Length( world->gravity );
	b3Vec3 worldUp = gravityLength > 0.0f ? b3MulSV( -1.0f / gravityLength, world->gravity ) : up;

	v->speed = b3Dot( linearVelocity, forward );

	// Shares are needed to know which wheels are driven
	float shares[B3_MAX_VEHICLE_WHEELS];
	b3ComputeTorqueShares( v, shares );

	bool blockUpshift = false;
	for ( int i = 0; i < wheelCount; ++i )
	{
		b3VehicleWheel* wheel = v->wheels + i;
		if ( shares[i] > 0.0f &&
			 ( wheel->hasContact == false || b3AbsFloat( wheel->slipRatio ) > 2.0f * wheel->def.tire.longitudinalPeakSlip ) )
		{
			blockUpshift = true;
		}
	}

	// Shift on the slower of the engine and the road speed, so wheels spinning in the air or in a
	// burnout do not shift up
	float shiftRpm = v->engineSpeed * B3_RADIANS_TO_RPM;
	{
		float radius = 0.0f;
		float count = 0.0f;
		for ( int i = 0; i < wheelCount; ++i )
		{
			if ( shares[i] > 0.0f )
			{
				radius += v->wheels[i].def.radius;
				count += 1.0f;
			}
		}

		if ( count > 0.0f )
		{
			float roadRpm = b3AbsFloat( v->speed * b3GetGearRatio( v ) ) * count / radius * B3_RADIANS_TO_RPM;
			shiftRpm = b3MinFloat( shiftRpm, roadRpm );
		}
	}

	b3UpdateGearbox( v, dt, shiftRpm, blockUpshift );
	float throttle = v->throttle;
	float brake = v->brake;
	float handbrake = input->handbrake;

	// Steering: speed sensitive, rate limited, with counter-steer into rear slides
	{
		const b3VehicleSteering* st = &v->steering;
		float speed = b3AbsFloat( v->speed );
		float maxAngle = st->maxAngle;
		if ( st->speedReference > 0.0f )
		{
			// Falls off with speed squared like the grip limited angle
			float x = speed / st->speedReference;
			maxAngle /= 1.0f + x * x;
		}
		float target = -input->steer * maxAngle;

		if ( v->assists.counterSteer > 0.0f && v->speed > 2.0f )
		{
			float rearOffset = 0.0f;
			int rearCount = 0;
			for ( int i = 0; i < wheelCount; ++i )
			{
				if ( v->wheels[i].def.steerFactor == 0.0f )
				{
					rearOffset += b3Dot( v->wheels[i].def.position, v->chassis.forward );
					rearCount += 1;
				}
			}
			rearOffset = rearCount > 0 ? rearOffset / rearCount : 0.0f;
			b3Vec3 rearVelocity = b3Add( linearVelocity, b3Cross( angularVelocity, b3MulSV( rearOffset, forward ) ) );
			float slide = b3Atan2( b3Dot( rearVelocity, left ), b3MaxFloat( b3Dot( rearVelocity, forward ), 0.1f ) );
			target += v->assists.counterSteer * slide;
		}

		target = b3ClampFloat( target, -st->maxAngle, st->maxAngle );
		bool outward = b3AbsFloat( target ) > b3AbsFloat( v->steer ) && target * v->steer >= 0.0f;
		float rate = outward ? st->steerSpeed : st->returnSpeed;
		float step = rate * dt;
		v->steer += b3ClampFloat( target - v->steer, -step, step );
	}

	// Ackermann geometry
	float frontOffset = 0.0f, rearOffset = 0.0f;
	int frontCount = 0, rearCount = 0;
	for ( int i = 0; i < wheelCount; ++i )
	{
		float z = b3Dot( v->wheels[i].def.position, v->chassis.forward );
		if ( v->wheels[i].def.steerFactor != 0.0f )
		{
			frontOffset += z;
			frontCount += 1;
		}
		else
		{
			rearOffset += z;
			rearCount += 1;
		}
	}
	float wheelBase = frontCount > 0 && rearCount > 0 ? frontOffset / frontCount - rearOffset / rearCount : 0.0f;
	b3Vec3 localLeft = b3Cross( v->chassis.up, v->chassis.forward );

	b3WheelFrame frames[B3_MAX_VEHICLE_WHEELS];
	int contactCount = 0;

	for ( int i = 0; i < wheelCount; ++i )
	{
		b3VehicleWheel* wheel = v->wheels + i;
		const b3VehicleWheelDef* def = &wheel->def;

		float angle = v->steer;
		if ( v->steering.ackermann > 0.0f && wheelBase > 0.0f && b3AbsFloat( angle ) > 1e-4f )
		{
			// Turn center on the rear axle line, offset to the side
			b3CosSin steerCs = b3ComputeCosSin( angle );
			float turnRadius = wheelBase * steerCs.cosine / steerCs.sine;
			float side = b3Dot( def->position, localLeft );
			float ackermann = b3Atan2( wheelBase, turnRadius - side );
			if ( angle < 0.0f )
			{
				ackermann -= B3_PI;
			}
			angle = b3LerpFloat( angle, ackermann, v->steering.ackermann );
		}
		wheel->steerAngle = def->steerFactor * angle;

		b3CosSin cs = b3ComputeCosSin( wheel->steerAngle );
		b3Vec3 heading = b3Add( b3MulSV( cs.cosine, forward ), b3MulSV( cs.sine, left ) );
		b3Vec3 axle = b3Cross( up, heading );

		b3CastWheel( world, v, wheel, xf, up, heading, axle, anchorId, frames + i );

		b3WheelFrame* frame = frames + i;
		if ( wheel->hasContact )
		{
			contactCount += 1;
			b3Vec3 n = wheel->contactNormal;
			b3Vec3 f = b3MulSub( heading, b3Dot( heading, n ), n );
			f = b3LengthSquared( f ) > 1e-6f ? b3Normalize( f ) : b3Perp( n );
			frame->forward = f;
			frame->lateral = b3Cross( n, f );

			b3Vec3 r = b3SubPos( wheel->contactPoint, centerOfMass );
			frame->rollOffset = b3MulSV( -v->assists.rollInfluence * b3Dot( r, up ), up );
		}
		else
		{
			frame->forward = heading;
			frame->lateral = left;
			frame->rollOffset = b3Vec3_zero;
		}
	}

	v->wheelsInContact = contactCount;

	// Anti-roll bars push on the compression difference
	float antiRollForce[B3_MAX_VEHICLE_WHEELS] = { 0 };
	for ( int i = 0; i < v->antiRoll.barCount; ++i )
	{
		const b3VehicleAntiRollBar* bar = v->antiRoll.bars + i;
		if ( bar->leftWheel < 0 || bar->rightWheel < 0 || bar->leftWheel == bar->rightWheel )
		{
			continue;
		}

		b3VehicleWheel* wl = v->wheels + bar->leftWheel;
		b3VehicleWheel* wr = v->wheels + bar->rightWheel;
		float cl = wl->hasContact ? wl->def.suspension.maxLength - wl->suspensionLength : 0.0f;
		float cr = wr->hasContact ? wr->def.suspension.maxLength - wr->suspensionLength : 0.0f;
		float force = bar->stiffness * ( cl - cr );
		antiRollForce[bar->leftWheel] += force;
		antiRollForce[bar->rightWheel] -= force;
	}

	// Slip, friction limits, brakes and sticky tires
	float frictionY[B3_MAX_VEHICLE_WHEELS];
	float gripX[B3_MAX_VEHICLE_WHEELS];
	float gripY[B3_MAX_VEHICLE_WHEELS];
	float peakTorque[B3_MAX_VEHICLE_WHEELS] = { 0 };
	float tractionTorque[B3_MAX_VEHICLE_WHEELS] = { 0 };
	b3Vec3 localCenter = b3Body_GetLocalCenter( v->chassisId );
	float gravity = b3GravityLength( world );
	for ( int i = 0; i < wheelCount; ++i )
	{
		b3VehicleWheel* wheel = v->wheels + i;
		const b3VehicleWheelDef* def = &wheel->def;
		const b3VehicleTire* tire = &def->tire;
		b3WheelFrame* frame = frames + i;

		float serviceTorque = brake * def->maxBrakeTorque;
		float brakeTorque = serviceTorque + handbrake * def->maxHandBrakeTorque;
		bool driven = shares[i] > 0.0f && throttle > 0.0f;

		frictionY[i] = 0.0f;
		gripX[i] = 0.0f;
		gripY[i] = 0.0f;
		bool wasSticky = wheel->isSticky;
		wheel->isSticky = false;

		if ( wheel->hasContact == false )
		{
			wheel->slipRatio = 0.0f;
			wheel->slipAngle = 0.0f;
			wheel->stickyTimer = 0.0f;
			wheel->brakeTorque = brakeTorque;
			continue;
		}

		// The load is the previous step's solver result, which is zero on the step a wheel touches
		// down and the first step after creation. The static weight stands in until then, so ABS and
		// traction control do not drop the brake and drive torque to zero on landing.
		float load = wheel->load > 0.0f ? wheel->load : wheel->sprungMass * gravity * frame->cosAngle;

		b3Vec3 vc = b3Body_GetWorldPointVelocity( v->chassisId, wheel->contactPoint );
		b3Vec3 vg = b3Vec3_zero;
		if ( B3_ID_EQUALS( frame->groundBodyId, anchorId ) == false )
		{
			vg = b3Body_GetWorldPointVelocity( frame->groundBodyId, wheel->contactPoint );
		}

		b3Vec3 vr = b3Sub( vc, vg );
		float vLong = b3Dot( vr, frame->forward );
		float vLat = b3Dot( vr, frame->lateral );

		float floor = driven || brakeTorque > 0.0f ? 0.1f : 4.0f;
		wheel->slipRatio = ( wheel->omega * def->radius - vLong ) / b3MaxFloat( b3AbsFloat( vLong ), floor );
		wheel->slipAngle = b3Atan2( vLat, b3AbsFloat( vLong ) + 1.0f );

		float ground = frame->groundScale;
		float muY = ground * b3TireCurve( b3AbsFloat( wheel->slipAngle ), tire->lateralPeakAngle, tire->lateralPeakFriction,
										  tire->lateralSlideAngle, tire->lateralSlideFriction );

		float drift = 1.0f;
		if ( def->maxHandBrakeTorque > 0.0f && handbrake > 0.0f )
		{
			drift = b3LerpFloat( 1.0f, v->assists.driftGripScale, handbrake );
		}

		muY *= drift;
		float peakX = ground * b3TireCurve( b3MaxFloat( b3AbsFloat( wheel->slipRatio ), tire->longitudinalPeakSlip ),
											 tire->longitudinalPeakSlip, tire->longitudinalPeakFriction,
											 tire->longitudinalSlideSlip, tire->longitudinalSlideFriction );
		float peakY = drift * ground *
					  b3TireCurve( b3MaxFloat( b3AbsFloat( wheel->slipAngle ), tire->lateralPeakAngle ), tire->lateralPeakAngle,
								   tire->lateralPeakFriction, tire->lateralSlideAngle, tire->lateralSlideFriction );

		// Sticky tire: pure stiction near rest so the car does not creep or jitter. Braking engages it
		// at once, idling after a short delay. It stays engaged up to twice the speed it engaged at.
		b3Vec3 planar = b3MulSub( vr, b3Dot( vr, wheel->contactNormal ), wheel->contactNormal );
		float stickySpeed = wasSticky ? 2.0f * v->assists.stickySpeed : v->assists.stickySpeed;
		bool slow = b3Length( planar ) < stickySpeed && b3AbsFloat( wheel->omega * def->radius ) < stickySpeed;
		bool holding = brake > 0.0f || handbrake > 0.0f;
		if ( v->assists.stickyTire && slow && ( throttle == 0.0f || handbrake > 0.0f ) )
		{
			wheel->stickyTimer += dt;
		}
		else
		{
			wheel->stickyTimer = 0.0f;
		}

		bool engage = wheel->stickyTimer > 0.0f && ( holding || wasSticky || wheel->stickyTimer >= v->assists.stickyTime );
		if ( engage )
		{
			wheel->isSticky = true;
			muY = ground * tire->lateralPeakFriction;
			peakX = ground * tire->longitudinalPeakFriction;
			peakY = muY;
			float holdTorque = b3MaxFloat( def->maxBrakeTorque, 2.0f * load * def->radius + 100.0f );
			brakeTorque = b3MaxFloat( brakeTorque, holdTorque );
		}

		// ABS caps the service brake at the torque the tire can transmit at peak grip, so the wheel
		// settles near the peak slip instead of locking
		if ( v->assists.abs && serviceTorque > 0.0f && wheel->isSticky == false )
		{
			float limit = ground * tire->longitudinalPeakFriction * load * def->radius;
			brakeTorque -= serviceTorque - b3MinFloat( serviceTorque, limit );
		}

		brakeTorque += tire->rollingResistance * load * def->radius;
		wheel->brakeTorque = brakeTorque;
		frictionY[i] = muY;
		gripX[i] = peakX;
		gripY[i] = peakY;
		peakTorque[i] = ground * tire->longitudinalPeakFriction * load * def->radius;

		// On wheels behind the center of mass, traction control leaves room on the friction ellipse
		// for the cornering force the tire carried last step, so full throttle mid corner does not
		// spin the car. Extra drive on the front wheels only widens the line, so they keep the full
		// peak. At least 30% of the peak is always allowed so the car can pull out of a corner.
		tractionTorque[i] = peakTorque[i];
		if ( b3Dot( b3Sub( def->position, localCenter ), v->chassis.forward ) < 0.0f )
		{
			float lateralCapacity = peakY * load;
			float lateralUse =
				lateralCapacity > 0.0f ? b3MinFloat( b3AbsFloat( wheel->lateralForce ) / lateralCapacity, 1.0f ) : 0.0f;
			tractionTorque[i] *= b3MaxFloat( sqrtf( 1.0f - lateralUse * lateralUse ), 0.3f );
		}

		// Past the peak slip the tire only transmits the sliding grip, which is less than the peak
		// torque above, so a wheel that already broke loose (for example from the load moving off it
		// faster than last step's load shows) would keep spinning. Cut its drive until it grips again.
		float driveSign = v->gear < 0 ? -1.0f : 1.0f;
		if ( driveSign * wheel->slipRatio > tire->longitudinalPeakSlip )
		{
			tractionTorque[i] = b3MinFloat( tractionTorque[i], 0.5f * peakX * load * def->radius );
		}
	}

	// Implicit engine, clutch and driven wheel solve. The ground reaction from the previous step
	// stands in for the tire load so the engine feels the car's inertia.
	{
		const b3VehicleEngine* e = &v->engine;
		const b3VehicleTransmission* t = &v->transmission;
		float ratio = b3GetGearRatio( v );
		float h = dt;

		float A = 0.0f;
		float b = 0.0f;
		float gearSpeed = 0.0f;
		for ( int i = 0; i < wheelCount; ++i )
		{
			if ( shares[i] == 0.0f )
			{
				continue;
			}

			b3VehicleWheel* wheel = v->wheels + i;
			float g = shares[i] * ratio;
			float invI = 1.0f / wheel->def.inertia;
			float groundTorque = wheel->hasContact ? -wheel->longitudinalForce * wheel->def.radius : 0.0f;
			A += g * g * invI;
			b += g * ( wheel->omega + h * groundTorque * invI );
			gearSpeed += g * wheel->omega;
		}

		float idle = e->idleRpm * B3_RPM_TO_RADIANS;
		float clutch = 1.0f;
		if ( v->gear == 0 || v->shiftTimer > 0.0f || A == 0.0f )
		{
			clutch = 0.0f;
		}
		else if ( v->clutchTimer > 0.0f && t->clutchReleaseTime > 0.0f )
		{
			clutch = 1.0f - v->clutchTimer / t->clutchReleaseTime;
		}

		// Open the clutch when coasting below idle so the engine does not stall or creep
		if ( throttle == 0.0f && gearSpeed < idle )
		{
			clutch = 0.0f;
		}

		float k = t->clutchStrength * clutch;
		float rpm = v->engineSpeed * B3_RADIANS_TO_RPM;

		// Traction control lowers the throttle until no grounded driven wheel gets more torque
		// than its tire can transmit at peak grip
		float engineThrottle = throttle;
		if ( v->assists.tractionControl && throttle > 0.0f && clutch > 0.0f )
		{
			float limit = FLT_MAX;
			for ( int i = 0; i < wheelCount; ++i )
			{
				float g = b3AbsFloat( shares[i] * ratio );
				if ( g > 0.0f && v->wheels[i].hasContact )
				{
					limit = b3MinFloat( limit, tractionTorque[i] / g );
				}
			}

			float full = b3EngineTorqueAt( e, rpm );
			if ( full > 0.0f && limit < FLT_MAX )
			{
				engineThrottle = b3MinFloat( throttle, limit / full );
			}
		}

		float engineTorque = engineThrottle * b3EngineTorqueAt( e, rpm );

		float rhs = e->inertia * v->engineSpeed + h * engineTorque;
		float a11 = e->inertia + h * k + h * e->angularDamping;
		float a12 = -h * k;
		float a21 = -h * k * A;
		float a22 = 1.0f + h * k * A;
		float det = a11 * a22 - a12 * a21;

		float engineSpeed = det > 0.0f ? ( rhs * a22 - a12 * b ) / det : v->engineSpeed;
		float wheelSpeed = det > 0.0f ? ( a11 * b - a21 * rhs ) / det : b;
		float clutchTorque = k * ( engineSpeed - wheelSpeed );

		// Below the floor the clutch slips. With throttle the floor rises toward a launch rpm.
		float launch = idle + 0.35f * ( e->maxRpm - e->idleRpm ) * B3_RPM_TO_RADIANS;
		float floorSpeed = b3LerpFloat( idle, launch, engineThrottle );
		if ( engineSpeed < floorSpeed )
		{
			engineSpeed = floorSpeed;
			wheelSpeed = ( b + h * k * A * floorSpeed ) / ( 1.0f + h * k * A );
			clutchTorque = k * ( floorSpeed - wheelSpeed );
			float available = engineThrottle * b3EngineTorqueAt( e, floorSpeed * B3_RADIANS_TO_RPM );
			clutchTorque = b3MinFloat( clutchTorque, available );
		}

		engineSpeed = b3MinFloat( engineSpeed, 1.05f * e->maxRpm * B3_RPM_TO_RADIANS );
		v->engineSpeed = b3IsValidFloat( engineSpeed ) ? engineSpeed : idle;
		if ( b3IsValidFloat( clutchTorque ) == false )
		{
			clutchTorque = 0.0f;
		}

		for ( int i = 0; i < wheelCount; ++i )
		{
			b3VehicleWheel* wheel = v->wheels + i;
			float torque = shares[i] * ratio * clutchTorque;

			// Drive torque has the sign of the gear ratio, so this also caps reverse. Engine braking
			// has the opposite sign and is left alone.
			if ( v->assists.tractionControl && wheel->hasContact && torque * ratio > 0.0f )
			{
				torque = b3MinFloat( b3AbsFloat( torque ), tractionTorque[i] ) * ( ratio > 0.0f ? 1.0f : -1.0f );
			}
			wheel->driveTorque = torque;
		}
	}

	// Write the wheel joints, retargeting them when the ground body changes
	for ( int i = 0; i < wheelCount; ++i )
	{
		b3VehicleWheel* wheel = v->wheels + i;
		const b3VehicleWheelDef* def = &wheel->def;
		const b3VehicleSuspension* s = &def->suspension;
		b3WheelFrame* frame = frames + i;

		// Wheel spin damping, implicit
		wheel->omega /= 1.0f + dt * def->angularDamping / def->inertia;

		b3VehicleWheelJoint carry = { 0 };
		bool valid = B3_IS_NON_NULL( wheel->jointId );
		if ( valid && B3_ID_EQUALS( wheel->groundBodyId, frame->groundBodyId ) == false )
		{
			b3Joint* joint = b3GetJointFullId( world, wheel->jointId );
			carry = b3GetJointSim( world, joint )->vehicleWheelJoint;
			b3DestroyJointInternal( world, joint, false );
			valid = false;
		}

		if ( valid == false )
		{
			wheel->jointId = b3CreateVehicleWheelJointInternal( world, v->chassisId, frame->groundBodyId );
			wheel->groundBodyId = frame->groundBodyId;
		}

		b3Joint* joint = b3GetJointFullId( world, wheel->jointId );
		b3JointSim* base = b3GetJointSim( world, joint );
		b3VehicleWheelJoint* sim = &base->vehicleWheelJoint;
		if ( valid == false )
		{
			sim->springImpulse = carry.springImpulse;
			sim->bumpImpulse = carry.bumpImpulse;
		}

		sim->omega = wheel->omega;
		sim->radius = def->radius;
		sim->invInertia = 1.0f / def->inertia;
		sim->driveTorque = wheel->driveTorque;
		sim->brakeTorque = wheel->brakeTorque;
		sim->hasContact = wheel->hasContact;

		if ( wheel->hasContact == false )
		{
			sim->springImpulse = 0.0f;
			sim->bumpImpulse = 0.0f;
			sim->lateralImpulse = 0.0f;
			sim->lateralFriction = 0.0f;
			sim->longitudinalGrip = 0.0f;
			sim->lateralGrip = 0.0f;
			continue;
		}

		base->localFrameA.p = b3Body_GetLocalPoint( v->chassisId, wheel->contactPoint );
		base->localFrameB.p = b3Body_GetLocalPoint( frame->groundBodyId, wheel->contactPoint );

		float cosAngle = frame->cosAngle;
		float springTarget = s->maxLength + s->preload;
		float stiffness = wheel->stiffness;
		float springC = ( wheel->suspensionLength - springTarget ) * cosAngle;
		if ( stiffness > 0.0f )
		{
			springC -= antiRollForce[i] / stiffness;
		}

		sim->normal = wheel->contactNormal;
		sim->forward = frame->forward;
		sim->lateral = frame->lateral;
		sim->rollOffset = frame->rollOffset;
		sim->springC = springC;
		sim->bumpC = ( wheel->suspensionLength - s->minLength ) * cosAngle;
		sim->stiffness = stiffness;
		sim->compressionDamping = wheel->compressionDamping;
		sim->reboundDamping = wheel->reboundDamping;
		sim->lateralFriction = frictionY[i];
		sim->longitudinalGrip = gripX[i];
		sim->lateralGrip = gripY[i];
		sim->enableBumpStop = s->enableBumpStop;
	}

	// Visual spin
	for ( int i = 0; i < wheelCount; ++i )
	{
		b3VehicleWheel* wheel = v->wheels + i;
		float angle = wheel->rotationAngle + wheel->omega * dt;
		if ( angle > B3_PI || angle < -B3_PI )
		{
			angle = b3UnwindAngle( angle );
		}
		wheel->rotationAngle = angle;
	}

	const b3VehicleAssists* assists = &v->assists;

	// Aerodynamics
	{
		float speed = b3Length( linearVelocity );
		b3Vec3 force = b3MulSV( -assists->drag * speed, linearVelocity );
		force = b3MulAdd( force, -assists->downforce * v->speed * v->speed, up );
		if ( b3LengthSquared( force ) > 0.0f )
		{
			b3Body_ApplyForceToCenter( v->chassisId, force, false );
		}
	}

	// Angular assists modify the chassis angular velocity directly
	b3Vec3 w = angularVelocity;
	bool modified = false;

	if ( contactCount == 0 && assists->airControl > 0.0f )
	{
		b3Vec3 accel = b3MulSV( input->forward, left );
		if ( handbrake > 0.5f )
		{
			accel = b3MulAdd( accel, input->steer, forward );
		}
		else
		{
			accel = b3MulAdd( accel, -input->steer, up );
		}
		w = b3MulAdd( w, dt * assists->airControl, accel );
		w = b3MulSV( 1.0f / ( 1.0f + dt * assists->airAngularDamping ), w );
		modified = true;
	}

	// Stuck: on the side or roof and nearly stopped
	float uprightness = b3Dot( up, worldUp );
	if ( uprightness < 0.3f && b3Length( linearVelocity ) < 1.0f )
	{
		v->stuckTimer += dt;
	}
	else
	{
		v->stuckTimer = 0.0f;
	}
	v->stuck = v->stuckTimer > 2.0f;

	if ( assists->selfRighting > 0.0f )
	{
		b3Vec3 u = b3MulSub( worldUp, b3Dot( worldUp, forward ), forward );
		if ( b3LengthSquared( u ) > 0.01f )
		{
			u = b3Normalize( u );
			float roll = b3Atan2( b3Dot( b3Cross( up, u ), forward ), b3Dot( up, u ) );
			bool partial = 0 < contactCount && contactCount < wheelCount && b3AbsFloat( roll ) > 0.3f;
			bool flipped = v->stuckTimer > 0.5f;
			if ( partial || flipped )
			{
				float stiffness = flipped ? 3.0f * assists->selfRighting : assists->selfRighting;
				float rollRate = b3Dot( w, forward );
				float accel = stiffness * roll - 2.0f * sqrtf( stiffness ) * rollRate;
				w = b3MulAdd( w, dt * accel, forward );
				modified = true;
			}
		}
	}

	if ( assists->maxTiltAngle > 0.0f && uprightness < b3ComputeCosSin( assists->maxTiltAngle ).cosine )
	{
		b3Vec3 axis = b3Cross( up, worldUp );
		if ( b3LengthSquared( axis ) > 1e-6f )
		{
			axis = b3Normalize( axis );
			float rate = b3Dot( w, axis );
			if ( rate < 0.0f )
			{
				w = b3MulSub( w, rate, axis );
				modified = true;
			}
		}
	}

	if ( modified && b3IsValidVec3( w ) )
	{
		b3Body_SetAngularVelocity( v->chassisId, w );
	}

	// Keep the chassis awake while the car is doing anything
	bool spinning = false;
	for ( int i = 0; i < wheelCount; ++i )
	{
		if ( b3AbsFloat( v->wheels[i].omega ) > 0.1745f )
		{
			spinning = true;
		}
	}

	bool revving = v->engineSpeed > 1.05f * v->engine.idleRpm * B3_RPM_TO_RADIANS;
	if ( hasInput || spinning || revving || v->shiftTimer > 0.0f )
	{
		body = b3GetBodyFullId( world, v->chassisId );
		body->sleepTime = 0.0f;
	}
}

void b3UpdateVehiclesPreStep( b3World* world, float timeStep )
{
	if ( world->vehicles.count == 0 || timeStep <= 0.0f )
	{
		return;
	}

	for ( int i = 0; i < world->vehicles.count; ++i )
	{
		b3Vehicle* v = world->vehicles.data + i;
		if ( v->id == B3_NULL_INDEX || v->generation == 0 )
		{
			continue;
		}

		b3UpdateVehicle( world, v, timeStep );
	}
}
