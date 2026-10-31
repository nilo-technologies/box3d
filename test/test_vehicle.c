// SPDX-FileCopyrightText: 2026 Nilo Technologies
// SPDX-License-Identifier: MIT

#include "test_macros.h"

#include "box3d/box3d.h"
#include "box3d/math_functions.h"

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define VEHICLE_DT ( 1.0f / 60.0f )
#define VEHICLE_SUBSTEPS 4
#define KMH ( 1.0f / 3.6f )

static b3WorldId CreateVehicleWorld( int workerCount )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = (b3Vec3){ 0.0f, -9.81f, 0.0f };
	worldDef.workerCount = workerCount;
	return b3CreateWorld( &worldDef );
}

static b3BodyId CreateGroundBox( b3WorldId worldId, b3Pos position, b3Quat rotation, b3Vec3 halfExtents, float friction )
{
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.position = position;
	bodyDef.rotation = rotation;
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
	b3BoxHull box = b3MakeBoxHull( halfExtents.x, halfExtents.y, halfExtents.z );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.baseMaterial.friction = friction;
	b3CreateHullShape( bodyId, &shapeDef, &box.base );
	return bodyId;
}

static b3BodyId CreateFlatGround( b3WorldId worldId )
{
	return CreateGroundBox( worldId, (b3Pos){ 0.0f, -1.0f, 0.0f }, b3Quat_identity, (b3Vec3){ 2000.0f, 1.0f, 2000.0f },
							0.6f );
}

static b3BodyId CreateChassis( b3WorldId worldId, b3Pos position, b3Quat rotation )
{
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = position;
	bodyDef.rotation = rotation;
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
	b3BoxHull box = b3MakeBoxHull( 0.9f, 0.3f, 2.1f );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateHullShape( bodyId, &shapeDef, &box.base );
	return bodyId;
}

static b3VehicleId CreateCar( b3WorldId worldId, b3Pos position, const b3VehicleDef* base )
{
	b3BodyId chassisId = CreateChassis( worldId, position, b3Quat_identity );
	b3VehicleDef def = base != NULL ? *base : b3DefaultVehicleDef();
	def.chassisId = chassisId;
	return b3CreateVehicle( worldId, &def );
}

static void Step( b3WorldId worldId, int count )
{
	for ( int i = 0; i < count; ++i )
	{
		b3World_Step( worldId, VEHICLE_DT, VEHICLE_SUBSTEPS );
	}
}

static bool IsFiniteTransform( b3WorldTransform xf )
{
	return b3IsValidPosition( xf.p ) && b3IsValidQuat( xf.q );
}

static bool VehicleIsFinite( b3VehicleId vehicleId )
{
	b3BodyId chassisId = b3Vehicle_GetChassisBody( vehicleId );
	if ( IsFiniteTransform( b3Body_GetTransform( chassisId ) ) == false )
	{
		return false;
	}

	if ( b3IsValidVec3( b3Body_GetLinearVelocity( chassisId ) ) == false ||
		 b3IsValidVec3( b3Body_GetAngularVelocity( chassisId ) ) == false )
	{
		return false;
	}

	b3VehicleState state = b3Vehicle_GetState( vehicleId );
	if ( b3IsValidFloat( state.engineRpm ) == false || b3IsValidFloat( state.speed ) == false )
	{
		return false;
	}

	int count = b3Vehicle_GetWheelCount( vehicleId );
	for ( int i = 0; i < count; ++i )
	{
		b3VehicleWheelState ws = b3Vehicle_GetWheelState( vehicleId, i );
		if ( b3IsValidFloat( ws.angularVelocity ) == false || b3IsValidFloat( ws.load ) == false ||
			 b3IsValidFloat( ws.slipRatio ) == false || b3IsValidFloat( ws.suspensionLength ) == false )
		{
			return false;
		}

		if ( IsFiniteTransform( b3Vehicle_GetWheelTransform( vehicleId, i ) ) == false )
		{
			return false;
		}
	}

	return true;
}

static float GetRollAngle( b3BodyId chassisId )
{
	b3Quat q = b3Body_GetRotation( chassisId );
	b3Vec3 up = b3RotateVector( q, b3Vec3_axisY );
	b3Vec3 forward = b3RotateVector( q, b3Vec3_axisZ );
	b3Vec3 left = b3Cross( up, forward );
	return asinf( b3ClampFloat( -left.y, -1.0f, 1.0f ) );
}

// Drops the car and checks the resting suspension and the time it takes to settle
static int RideHeightTest( void )
{
	b3WorldId worldId = CreateVehicleWorld( 1 );
	CreateFlatGround( worldId );
	b3VehicleId vehicleId = CreateCar( worldId, (b3Pos){ 0.0f, 1.0f, 0.0f }, NULL );
	ENSURE( b3Vehicle_IsValid( vehicleId ) );
	b3BodyId chassisId = b3Vehicle_GetChassisBody( vehicleId );

	// Sprung masses sum to the chassis mass
	float total = 0.0f;
	for ( int i = 0; i < 4; ++i )
	{
		total += b3Vehicle_GetSprungMass( vehicleId, i );
	}
	ENSURE_SMALL( total - 1300.0f, 1.0f );

	float settleTime = -1.0f;
	for ( int i = 0; i < 300; ++i )
	{
		Step( worldId, 1 );
		float vy = b3Body_GetLinearVelocity( chassisId ).y;
		if ( fabsf( vy ) > 0.02f )
		{
			settleTime = -1.0f;
		}
		else if ( settleTime < 0.0f )
		{
			settleTime = ( i + 1 ) * VEHICLE_DT;
		}
	}

	float height = (float)b3Body_GetPosition( chassisId ).y;
	printf( "    ride height %.3f, settle %.2f s\n", height, settleTime );

	ENSURE( settleTime > 0.0f && settleTime < 2.5f );
	ENSURE( VehicleIsFinite( vehicleId ) );

	b3VehicleState state = b3Vehicle_GetState( vehicleId );
	ENSURE( state.wheelsInContact == 4 );

	for ( int i = 0; i < 4; ++i )
	{
		b3VehicleWheelState ws = b3Vehicle_GetWheelState( vehicleId, i );
		printf( "    wheel %d compression %.2f load %.0f\n", i, ws.compression, ws.load );
		ENSURE( ws.hasContact );
		ENSURE( 0.2f < ws.compression && ws.compression < 0.6f );
		ENSURE( ws.load > 2000.0f && ws.load < 5000.0f );
		ENSURE( fabsf( ws.angularVelocity ) < 0.1f );
	}

	b3Vec3 v = b3Body_GetLinearVelocity( chassisId );
	ENSURE( b3Length( v ) < 0.01f );

	b3DestroyWorld( worldId );
	return 0;
}

static float AccelerateTo( b3WorldId worldId, b3VehicleId vehicleId, float targetSpeed, float maxTime, int* maxGear,
						   bool* gearsInOrder )
{
	b3Vehicle_SetInput( vehicleId, (b3VehicleInput){ .forward = 1.0f } );
	int lastGear = b3Vehicle_GetState( vehicleId ).gear;
	*gearsInOrder = true;
	float time = 0.0f;
	while ( time < maxTime )
	{
		Step( worldId, 1 );
		time += VEHICLE_DT;
		b3VehicleState state = b3Vehicle_GetState( vehicleId );
		if ( state.gear < lastGear )
		{
			*gearsInOrder = false;
		}
		if ( state.gear > lastGear + 1 )
		{
			*gearsInOrder = false;
		}
		lastGear = state.gear;
		*maxGear = state.gear > *maxGear ? state.gear : *maxGear;
		if ( state.speed >= targetSpeed )
		{
			return time;
		}
	}
	return -1.0f;
}

static int AccelerationTest( void )
{
	b3WorldId worldId = CreateVehicleWorld( 1 );
	CreateFlatGround( worldId );
	b3VehicleId vehicleId = CreateCar( worldId, (b3Pos){ 0.0f, 0.6f, 0.0f }, NULL );
	Step( worldId, 120 );

	int maxGear = 0;
	bool inOrder = true;
	float time = AccelerateTo( worldId, vehicleId, 100.0f * KMH, 30.0f, &maxGear, &inOrder );
	printf( "    0-100 km/h %.2f s, gear %d\n", time, maxGear );
	ENSURE( 4.0f < time && time < 11.0f );
	ENSURE( maxGear >= 3 );
	ENSURE( inOrder );
	ENSURE( VehicleIsFinite( vehicleId ) );

	// Keeps driving straight
	b3Vec3 p = b3ToVec3( b3Body_GetPosition( b3Vehicle_GetChassisBody( vehicleId ) ) );
	ENSURE( fabsf( p.x ) < 1.0f );

	b3DestroyWorld( worldId );
	return 0;
}

// A full throttle launch from rest, with and without traction control. forward is 1 to launch
// forward and -1 to launch in reverse. Returns the largest driven wheel slip in that direction.
// Launch control is off so this measures traction control alone.
static float LaunchSlip( bool tractionControl, float forward )
{
	b3WorldId worldId = CreateVehicleWorld( 1 );
	CreateFlatGround( worldId );
	b3VehicleDef def = b3DefaultVehicleDef();
	def.assists.tractionControl = tractionControl;
	def.assists.launchControl = false;
	b3VehicleId vehicleId = CreateCar( worldId, (b3Pos){ 0.0f, 0.6f, 0.0f }, &def );
	Step( worldId, 120 );

	b3Vehicle_SetInput( vehicleId, (b3VehicleInput){ .forward = forward } );
	float maxSlip = 0.0f;
	for ( int i = 0; i < 120; ++i )
	{
		Step( worldId, 1 );
		for ( int j = 2; j < 4; ++j )
		{
			float sr = b3Vehicle_GetWheelState( vehicleId, j ).slipRatio;
			maxSlip = b3MaxFloat( maxSlip, forward * sr );
		}
	}

	b3DestroyWorld( worldId );
	return maxSlip;
}

// Full throttle at full lock from 70 km/h. Returns the largest chassis sideslip angle in degrees.
// Launch control is off so it cannot still be active when the corner starts.
static float PowerOversteer( bool tractionControl )
{
	b3WorldId worldId = CreateVehicleWorld( 1 );
	CreateFlatGround( worldId );
	b3VehicleDef def = b3DefaultVehicleDef();
	def.assists.tractionControl = tractionControl;
	def.assists.launchControl = false;
	b3VehicleId vehicleId = CreateCar( worldId, (b3Pos){ 0.0f, 0.6f, 0.0f }, &def );
	b3BodyId chassisId = b3Vehicle_GetChassisBody( vehicleId );
	Step( worldId, 60 );

	int maxGear = 0;
	bool inOrder;
	float maxSideslip = -1.0f;
	if ( AccelerateTo( worldId, vehicleId, 70.0f * KMH, 30.0f, &maxGear, &inOrder ) > 0.0f )
	{
		maxSideslip = 0.0f;
		b3Vehicle_SetInput( vehicleId, (b3VehicleInput){ .forward = 1.0f, .steer = 1.0f } );
		for ( int i = 0; i < 240; ++i )
		{
			Step( worldId, 1 );
			b3Vec3 v = b3Body_GetLinearVelocity( chassisId );
			b3Vec3 forward = b3RotateVector( b3Body_GetRotation( chassisId ), b3Vec3_axisZ );
			float speed = sqrtf( v.x * v.x + v.z * v.z );
			if ( speed > 1.0f )
			{
				float sideslip = atan2f( forward.x * v.z - forward.z * v.x, forward.x * v.x + forward.z * v.z );
				maxSideslip = b3MaxFloat( maxSideslip, fabsf( sideslip ) * 180.0f / B3_PI );
			}
		}
	}

	b3DestroyWorld( worldId );
	return maxSideslip;
}

static int TractionControlTest( void )
{
	float with = LaunchSlip( true, 1.0f );
	float without = LaunchSlip( false, 1.0f );
	printf( "    launch slip ratio %.2f with traction control, %.2f without\n", with, without );
	ENSURE( with < 0.15f );
	ENSURE( without > 1.0f );

	// Reverse drive torque is negative, the cap must still apply. Launching backward moves load off
	// the driven rear wheels faster than the cap sees it, so a wheel can break loose for a step
	// before traction control cuts its drive.
	float reverseWith = LaunchSlip( true, -1.0f );
	float reverseWithout = LaunchSlip( false, -1.0f );
	printf( "    reverse launch slip ratio %.2f with traction control, %.2f without\n", reverseWith, reverseWithout );
	ENSURE( reverseWith < 0.3f );
	ENSURE( reverseWithout > 1.0f );

	float cornerWith = PowerOversteer( true );
	float cornerWithout = PowerOversteer( false );
	printf( "    full throttle at full lock: sideslip %.1f deg with traction control, %.1f deg without\n", cornerWith,
			cornerWithout );
	ENSURE( cornerWith >= 0.0f && cornerWith < 15.0f );
	ENSURE( cornerWithout > 30.0f );
	return 0;
}

typedef struct LaunchResult
{
	float maxSlip;
	float launchSlip;
	float time60;
	bool sawLaunching;
	bool endedWhileHeld;
	bool clearedOnLift;
	bool finite;
} LaunchResult;

// Full throttle from rest for 8 s, then a lift. Records the largest driven wheel slip ratio over
// the run and while launch control is active, the time to 60 km/h and what the launch control
// state did. Launch control hands back once the tires grip, so a gear change later on can still
// spin them for a moment when traction control is off.
static LaunchResult LaunchRun( bool launchControl, bool tractionControl )
{
	b3WorldId worldId = CreateVehicleWorld( 1 );
	CreateFlatGround( worldId );
	b3VehicleDef def = b3DefaultVehicleDef();
	def.assists.launchControl = launchControl;
	def.assists.tractionControl = tractionControl;
	b3VehicleId vehicleId = CreateCar( worldId, (b3Pos){ 0.0f, 0.6f, 0.0f }, &def );
	Step( worldId, 120 );

	LaunchResult result = { 0.0f, 0.0f, -1.0f, false, false, false, false };
	b3Vehicle_SetInput( vehicleId, (b3VehicleInput){ .forward = 1.0f } );
	for ( int i = 0; i < 480; ++i )
	{
		Step( worldId, 1 );
		b3VehicleState state = b3Vehicle_GetState( vehicleId );
		for ( int j = 2; j < 4; ++j )
		{
			float slip = b3Vehicle_GetWheelState( vehicleId, j ).slipRatio;
			result.maxSlip = b3MaxFloat( result.maxSlip, slip );
			if ( state.isLaunching )
			{
				result.launchSlip = b3MaxFloat( result.launchSlip, slip );
			}
		}

		result.sawLaunching = result.sawLaunching || state.isLaunching;
		result.endedWhileHeld = result.sawLaunching && state.isLaunching == false;
		if ( result.time60 < 0.0f && state.speed >= 60.0f * KMH )
		{
			result.time60 = ( i + 1 ) * VEHICLE_DT;
		}
	}

	b3Vehicle_SetInput( vehicleId, (b3VehicleInput){ 0 } );
	Step( worldId, 1 );
	result.clearedOnLift = b3Vehicle_GetState( vehicleId ).isLaunching == false;
	result.finite = VehicleIsFinite( vehicleId );
	b3DestroyWorld( worldId );
	return result;
}

// Launch control holds the driven tires near their peak slip from rest, with traction control on
// or off, and hands back once the tires grip
static int LaunchControlTest( void )
{
	b3VehicleDef def = b3DefaultVehicleDef();
	ENSURE( def.assists.launchControl );
	float peakSlip = def.wheels[2].tire.longitudinalPeakSlip;

	LaunchResult with = LaunchRun( true, false );
	LaunchResult without = LaunchRun( false, false );
	printf( "    no traction control: launch slip %.2f with launch control, %.2f without (peak %.2f)\n", with.launchSlip,
			without.maxSlip, peakSlip );
	ENSURE( with.launchSlip < 2.0f * peakSlip );
	ENSURE( without.maxSlip > 1.0f );
	ENSURE( with.finite && without.finite );
	ENSURE( with.sawLaunching && with.endedWhileHeld && with.clearedOnLift );
	ENSURE( without.sawLaunching == false );

	// Traction control alone leaves grip unused on a launch; launch control is no slower
	LaunchResult both = LaunchRun( true, true );
	LaunchResult tractionOnly = LaunchRun( false, true );
	printf( "    0-60 km/h %.2f s with launch control, %.2f s with traction control alone\n", both.time60,
			tractionOnly.time60 );
	ENSURE( both.finite && tractionOnly.finite );
	ENSURE( both.time60 > 0.0f && tractionOnly.time60 > 0.0f );
	ENSURE( both.time60 <= tractionOnly.time60 );
	return 0;
}

// The slip ratio builds up over the relaxation length instead of jumping when the car pulls away,
// and at speed it settles at the plain ratio
static int SlipRelaxationTest( void )
{
	float early[2];
	float relaxations[2] = { 0.0f, 0.3f };
	for ( int k = 0; k < 2; ++k )
	{
		b3WorldId worldId = CreateVehicleWorld( 1 );
		CreateFlatGround( worldId );
		b3VehicleDef def = b3DefaultVehicleDef();
		def.assists.launchControl = false;
		def.assists.tractionControl = false;
		for ( int j = 0; j < def.wheelCount; ++j )
		{
			def.wheels[j].tire.longitudinalRelaxationLength = relaxations[k];
		}
		b3VehicleId vehicleId = CreateCar( worldId, (b3Pos){ 0.0f, 0.6f, 0.0f }, &def );
		Step( worldId, 120 );

		// The largest driven slip over the first 0.05 s of full throttle
		early[k] = 0.0f;
		b3Vehicle_SetInput( vehicleId, (b3VehicleInput){ .forward = 1.0f } );
		for ( int i = 0; i < 3; ++i )
		{
			Step( worldId, 1 );
			for ( int j = 2; j < 4; ++j )
			{
				early[k] = b3MaxFloat( early[k], b3Vehicle_GetWheelState( vehicleId, j ).slipRatio );
			}
		}

		// Cruise, then compare the reported slip with the plain ratio
		if ( k == 1 )
		{
			b3Vehicle_SetInput( vehicleId, (b3VehicleInput){ .forward = 0.3f } );
			Step( worldId, 360 );
			float speed = b3Vehicle_GetState( vehicleId ).speed;
			b3VehicleWheelState ws = b3Vehicle_GetWheelState( vehicleId, 2 );
			float plain = ( ws.angularVelocity * def.wheels[2].radius - speed ) / speed;
			printf( "    at %.1f m/s: slip %.4f, plain ratio %.4f\n", speed, ws.slipRatio, plain );
			ENSURE( speed > 10.0f );
			ENSURE( fabsf( ws.slipRatio - plain ) < 0.02f );
		}

		ENSURE( VehicleIsFinite( vehicleId ) );
		b3DestroyWorld( worldId );
	}

	printf( "    slip 0.05 s into a full throttle launch: %.2f instant, %.2f relaxed\n", early[0], early[1] );
	ENSURE( early[0] > 1.0f );
	ENSURE( early[1] < 0.5f * early[0] );
	return 0;
}

static float BrakingDistance( bool abs )
{
	b3WorldId worldId = CreateVehicleWorld( 1 );
	CreateFlatGround( worldId );
	b3VehicleDef def = b3DefaultVehicleDef();
	def.assists.abs = abs;
	b3VehicleId vehicleId = CreateCar( worldId, (b3Pos){ 0.0f, 0.6f, 0.0f }, &def );
	b3BodyId chassisId = b3Vehicle_GetChassisBody( vehicleId );
	Step( worldId, 60 );

	int maxGear = 0;
	bool inOrder;
	float time = AccelerateTo( worldId, vehicleId, 100.0f * KMH, 30.0f, &maxGear, &inOrder );
	if ( time < 0.0f )
	{
		b3DestroyWorld( worldId );
		return -1.0f;
	}

	b3Vec3 start = b3ToVec3( b3Body_GetPosition( chassisId ) );
	b3Vehicle_SetInput( vehicleId, (b3VehicleInput){ .brake = 1.0f } );
	float elapsed = 0.0f;
	while ( b3Vehicle_GetState( vehicleId ).speed > 0.05f && elapsed < 20.0f )
	{
		Step( worldId, 1 );
		elapsed += VEHICLE_DT;
	}

	bool finite = VehicleIsFinite( vehicleId );
	b3Vec3 end = b3ToVec3( b3Body_GetPosition( chassisId ) );
	b3DestroyWorld( worldId );
	return finite ? b3Distance( start, end ) : -1.0f;
}

static int BrakingTest( void )
{
	float plain = BrakingDistance( false );
	float abs = BrakingDistance( true );
	printf( "    100-0 km/h braking %.1f m, with abs %.1f m\n", plain, abs );
	// v^2 / (2 mu g) is 41 m on the sliding friction and 33 m on the peak friction
	ENSURE( 33.0f < plain && plain < 45.0f );
	ENSURE( 28.0f < abs && abs < plain );
	return 0;
}

// Full brake and full lock from 90 km/h for 1.5 s: the share of steps with a front wheel locked
// and the heading change
static float BrakeTurn( bool abs, float* heading )
{
	b3WorldId worldId = CreateVehicleWorld( 1 );
	CreateFlatGround( worldId );
	b3VehicleDef def = b3DefaultVehicleDef();
	def.assists.abs = abs;
	b3VehicleId vehicleId = CreateCar( worldId, (b3Pos){ 0.0f, 0.6f, 0.0f }, &def );
	b3BodyId chassisId = b3Vehicle_GetChassisBody( vehicleId );
	Step( worldId, 60 );

	int maxGear = 0;
	bool inOrder;
	if ( AccelerateTo( worldId, vehicleId, 90.0f * KMH, 30.0f, &maxGear, &inOrder ) < 0.0f )
	{
		*heading = 0.0f;
		b3DestroyWorld( worldId );
		return -1.0f;
	}

	b3Vec3 forward = b3RotateVector( b3Body_GetRotation( chassisId ), b3Vec3_axisZ );
	float yaw0 = atan2f( forward.x, forward.z );
	b3Vehicle_SetInput( vehicleId, (b3VehicleInput){ .brake = 1.0f, .steer = 1.0f } );
	int locked = 0;
	int steps = 90;
	for ( int i = 0; i < steps; ++i )
	{
		Step( worldId, 1 );
		for ( int j = 0; j < 2; ++j )
		{
			if ( b3Vehicle_GetWheelState( vehicleId, j ).slipRatio < -0.5f )
			{
				locked += 1;
				break;
			}
		}
	}

	forward = b3RotateVector( b3Body_GetRotation( chassisId ), b3Vec3_axisZ );
	float turn = atan2f( forward.x, forward.z ) - yaw0;
	*heading = fabsf( atan2f( sinf( turn ), cosf( turn ) ) );
	bool finite = VehicleIsFinite( vehicleId );
	b3DestroyWorld( worldId );
	return finite ? (float)locked / steps : -1.0f;
}

// ABS leaves room for the cornering force, so braking at full lock keeps the front wheels rolling
// and the car turns. Without ABS the fronts lock and the car slides on.
static int BrakeTurnTest( void )
{
	float plainHeading, absHeading;
	float plain = BrakeTurn( false, &plainHeading );
	float abs = BrakeTurn( true, &absHeading );
	printf( "    brake at full lock: fronts locked %.0f%% of the time, turned %.0f deg; with abs %.0f%%, %.0f deg\n",
			100.0f * plain, plainHeading * B3_RAD_TO_DEG, 100.0f * abs, absHeading * B3_RAD_TO_DEG );
	ENSURE( plain > 0.5f );
	ENSURE( 0.0f <= abs && abs < 0.1f );
	ENSURE( absHeading > 1.2f * plainHeading );
	return 0;
}

static float TurnRadius( const b3VehicleDef* def, float steer, float speed, float* maxRoll )
{
	b3WorldId worldId = CreateVehicleWorld( 1 );
	CreateFlatGround( worldId );
	b3VehicleId vehicleId = CreateCar( worldId, (b3Pos){ 0.0f, 0.6f, 0.0f }, def );
	b3BodyId chassisId = b3Vehicle_GetChassisBody( vehicleId );
	Step( worldId, 60 );

	// Simple speed controller
	float radius = -1.0f;
	*maxRoll = 0.0f;
	for ( int i = 0; i < 900; ++i )
	{
		b3VehicleState state = b3Vehicle_GetState( vehicleId );
		float forward = b3ClampFloat( 0.5f * ( speed - state.speed ), -1.0f, 1.0f );
		bool turning = i > 240;
		b3Vehicle_SetInput( vehicleId, (b3VehicleInput){ .forward = forward, .steer = turning ? steer : 0.0f } );
		Step( worldId, 1 );

		if ( i > 600 )
		{
			b3Vec3 v = b3Body_GetLinearVelocity( chassisId );
			b3Vec3 w = b3Body_GetAngularVelocity( chassisId );
			float vh = sqrtf( v.x * v.x + v.z * v.z );
			radius = fabsf( w.y ) > 1e-3f ? vh / fabsf( w.y ) : FLT_MAX;
			float roll = fabsf( GetRollAngle( chassisId ) );
			*maxRoll = b3MaxFloat( *maxRoll, roll );
		}
	}

	if ( VehicleIsFinite( vehicleId ) == false )
	{
		radius = -1.0f;
	}

	b3DestroyWorld( worldId );
	return radius;
}

static int TurningTest( void )
{
	b3VehicleDef def = b3DefaultVehicleDef();
	float roll;
	float radius = TurnRadius( &def, 1.0f, 5.0f, &roll );
	printf( "    turning radius at 5 m/s full lock %.2f m\n", radius );

	// Geometric radius is about wheelbase / tan(steer)
	ENSURE( 3.5f < radius && radius < 9.0f );

	float fastRadius = TurnRadius( &def, 0.5f, 15.0f, &roll );
	printf( "    turning radius at 15 m/s half lock %.2f m\n", fastRadius );
	// Grip limited: the lateral acceleration stays below the peak friction
	float lateralAcceleration = 15.0f * 15.0f / fastRadius;
	ENSURE( 15.0f < fastRadius && fastRadius < 60.0f );
	ENSURE( lateralAcceleration < 1.15f * 9.81f );
	return 0;
}

static int AntiRollTest( void )
{
	b3VehicleDef def = b3DefaultVehicleDef();
	float rollWith, rollWithout;
	TurnRadius( &def, 0.6f, 15.0f, &rollWith );

	def.antiRoll.barCount = 0;
	TurnRadius( &def, 0.6f, 15.0f, &rollWithout );

	printf( "    body roll %.2f deg with bars, %.2f deg without\n", rollWith * 180.0f / B3_PI,
			rollWithout * 180.0f / B3_PI );
	ENSURE( rollWith < 0.8f * rollWithout );
	return 0;
}

static float SlopeDrift( float angleDegrees, bool sideways, float handbrake )
{
	b3WorldId worldId = CreateVehicleWorld( 1 );
	float angle = angleDegrees * B3_PI / 180.0f;
	b3Vec3 axis = sideways ? b3Vec3_axisZ : b3Vec3_axisX;
	b3Quat q = b3MakeQuatFromAxisAngle( axis, angle );
	CreateGroundBox( worldId, (b3Pos){ 0.0f, -1.0f, 0.0f }, q, (b3Vec3){ 100.0f, 1.0f, 100.0f }, 0.6f );

	b3BodyId chassisId = CreateChassis( worldId, (b3Pos){ 0.0f, 0.7f, 0.0f }, q );
	b3VehicleDef def = b3DefaultVehicleDef();
	def.chassisId = chassisId;
	b3VehicleId vehicleId = b3CreateVehicle( worldId, &def );
	b3Vehicle_SetInput( vehicleId, (b3VehicleInput){ .handbrake = handbrake } );

	// Settle, then measure the drift
	Step( worldId, 120 );
	b3Vec3 start = b3ToVec3( b3Body_GetPosition( chassisId ) );
	Step( worldId, 300 );
	b3Vec3 end = b3ToVec3( b3Body_GetPosition( chassisId ) );

	float drift = VehicleIsFinite( vehicleId ) ? b3Distance( start, end ) : FLT_MAX;
	b3DestroyWorld( worldId );
	return drift;
}

static int SlopeHoldTest( void )
{
	float handbrake = SlopeDrift( 20.0f, false, 1.0f );
	float side = SlopeDrift( 15.0f, true, 0.0f );
	float rest = SlopeDrift( 10.0f, false, 0.0f );
	float steep = SlopeDrift( 20.0f, false, 0.0f );
	printf( "    drift: handbrake 20 deg %.4f m, side 15 deg %.4f m, idle 10 deg %.4f m, idle 20 deg %.4f m\n", handbrake,
			side, rest, steep );
	ENSURE( handbrake < 0.05f );
	ENSURE( side < 0.05f );
	ENSURE( rest < 0.05f );
	ENSURE( steep < 0.05f );
	return 0;
}

// A kinematic platform carries the car, then the car drives onto a dynamic slab and off it
static int DynamicGroundTest( void )
{
	b3WorldId worldId = CreateVehicleWorld( 1 );
	CreateFlatGround( worldId );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_kinematicBody;
	bodyDef.position = (b3Pos){ 0.0f, 0.25f, 0.0f };
	b3BodyId platformId = b3CreateBody( worldId, &bodyDef );
	b3BoxHull box = b3MakeBoxHull( 3.0f, 0.25f, 4.0f );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateHullShape( platformId, &shapeDef, &box.base );

	b3VehicleId vehicleId = CreateCar( worldId, (b3Pos){ 0.0f, 1.1f, 0.0f }, NULL );
	b3BodyId chassisId = b3Vehicle_GetChassisBody( vehicleId );
	Step( worldId, 120 );

	b3Body_SetLinearVelocity( platformId, (b3Vec3){ 1.0f, 0.0f, 0.0f } );
	Step( worldId, 120 );
	b3Vec3 v = b3Body_GetLinearVelocity( chassisId );
	printf( "    carried velocity %.3f %.3f %.3f\n", v.x, v.y, v.z );
	ENSURE( fabsf( v.x - 1.0f ) < 0.05f );
	ENSURE( b3Vehicle_GetWheelState( vehicleId, 0 ).hasContact );
	b3Body_SetLinearVelocity( platformId, b3Vec3_zero );

	// Dynamic slab in front. Its contact force with the ground grows by the car's weight.
	bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	b3Vec3 carPosition = b3ToVec3( b3Body_GetPosition( chassisId ) );
	bodyDef.position = (b3Pos){ carPosition.x, 0.1f, 12.0f };
	b3BodyId slabId = b3CreateBody( worldId, &bodyDef );
	b3BoxHull slab = b3MakeBoxHull( 3.0f, 0.1f, 4.0f );
	shapeDef = b3DefaultShapeDef();
	shapeDef.density = 50.0f;
	shapeDef.baseMaterial.friction = 0.8f;
	b3CreateHullShape( slabId, &shapeDef, &slab.base );

	float slabMass = b3Body_GetMass( slabId );

	// Drive off the platform onto the slab, stop on it
	bool onSlab = false;
	float slabPush = 0.0f;
	for ( int i = 0; i < 600; ++i )
	{
		b3Vec3 p = b3ToVec3( b3Body_GetPosition( chassisId ) );
		float forward = p.z < 12.0f ? 0.4f : -1.0f;
		if ( onSlab )
		{
			forward = 0.0f;
		}
		b3Vehicle_SetInput( vehicleId, (b3VehicleInput){ .forward = forward, .brake = onSlab ? 1.0f : 0.0f } );
		Step( worldId, 1 );

		if ( fabsf( p.z - 12.0f ) < 1.0f && fabsf( b3Vehicle_GetState( vehicleId ).speed ) < 0.5f )
		{
			onSlab = true;
		}

		if ( onSlab )
		{
			b3VehicleWheelState ws = b3Vehicle_GetWheelState( vehicleId, 0 );
			if ( ws.hasContact && b3Shape_GetBody( ws.groundShapeId ).index1 == slabId.index1 )
			{
				slabPush = b3MaxFloat( slabPush, ws.load );
			}
		}
	}

	printf( "    on slab %d, slab wheel load %.0f N, slab mass %.0f kg\n", onSlab, slabPush, slabMass );
	ENSURE( onSlab );
	ENSURE( slabPush > 1500.0f );

	// The slab sits on the ground, so it should not have been launched
	b3Vec3 slabVelocity = b3Body_GetLinearVelocity( slabId );
	ENSURE( b3Length( slabVelocity ) < 0.5f );

	// Drive off the slab back onto static ground
	b3Vehicle_SetInput( vehicleId, (b3VehicleInput){ .forward = 0.5f } );
	Step( worldId, 300 );
	b3VehicleWheelState ws = b3Vehicle_GetWheelState( vehicleId, 0 );
	ENSURE( ws.hasContact );
	ENSURE( b3Shape_GetBody( ws.groundShapeId ).index1 != slabId.index1 );
	ENSURE( VehicleIsFinite( vehicleId ) );

	// Destroying the ground body under a wheel must not break the vehicle
	b3Vehicle_SetInput( vehicleId, (b3VehicleInput){ 0 } );
	b3DestroyBody( slabId );
	Step( worldId, 60 );
	ENSURE( VehicleIsFinite( vehicleId ) );

	b3DestroyWorld( worldId );
	return 0;
}

// Hits a ramp: no upshift on landing and the bump stop keeps the wheels out of the ground
static int RampJumpTest( void )
{
	b3WorldId worldId = CreateVehicleWorld( 1 );
	CreateFlatGround( worldId );
	b3Quat q = b3MakeQuatFromAxisAngle( b3Vec3_axisX, -15.0f * B3_PI / 180.0f );
	CreateGroundBox( worldId, (b3Pos){ 0.0f, -0.4f, 60.0f }, q, (b3Vec3){ 4.0f, 1.0f, 6.0f }, 0.6f );

	b3VehicleId vehicleId = CreateCar( worldId, (b3Pos){ 0.0f, 0.6f, 0.0f }, NULL );
	b3BodyId chassisId = b3Vehicle_GetChassisBody( vehicleId );
	Step( worldId, 60 );

	b3Vehicle_SetInput( vehicleId, (b3VehicleInput){ .forward = 1.0f } );
	bool airborne = false;
	bool landed = false;
	int gearAtTakeoff = 0;
	int landingTimer = 0;
	bool upshiftOnLanding = false;
	float minLength = FLT_MAX;
	float minHeight = FLT_MAX;
	float maxHeight = 0.0f;

	for ( int i = 0; i < 900; ++i )
	{
		Step( worldId, 1 );
		b3VehicleState state = b3Vehicle_GetState( vehicleId );
		float y = (float)b3Body_GetPosition( chassisId ).y;
		maxHeight = b3MaxFloat( maxHeight, y );

		if ( state.wheelsInContact == 0 && airborne == false && state.speed > 10.0f )
		{
			airborne = true;
			gearAtTakeoff = state.gear;
		}

		if ( airborne && landed == false && state.wheelsInContact == 4 )
		{
			landed = true;
			landingTimer = 30;
		}

		if ( landingTimer > 0 )
		{
			landingTimer -= 1;
			if ( state.gear > gearAtTakeoff )
			{
				upshiftOnLanding = true;
			}
		}

		if ( landed )
		{
			minHeight = b3MinFloat( minHeight, y );
			for ( int j = 0; j < 4; ++j )
			{
				b3VehicleWheelState ws = b3Vehicle_GetWheelState( vehicleId, j );
				if ( ws.hasContact )
				{
					minLength = b3MinFloat( minLength, ws.suspensionLength );
				}
			}
		}

		if ( landed && landingTimer == 0 )
		{
			break;
		}
	}

	printf( "    airborne %d landed %d, max height %.2f, landing min height %.2f, min length %.3f\n", airborne, landed,
			maxHeight, minHeight, minLength );
	ENSURE( airborne && landed );
	ENSURE( upshiftOnLanding == false );

	b3VehicleWheelDef wheel = b3Vehicle_GetWheel( vehicleId, 0 );
	ENSURE( minLength > wheel.suspension.minLength - 0.1f );

	// Chassis bottom stays above the ground
	ENSURE( minHeight > 0.3f );
	ENSURE( VehicleIsFinite( vehicleId ) );

	b3DestroyWorld( worldId );
	return 0;
}

static int RunNanScenario( const b3VehicleDef* def, b3Pos position, b3Quat rotation, b3VehicleInput input )
{
	b3WorldId worldId = CreateVehicleWorld( 1 );
	CreateFlatGround( worldId );
	b3BodyId chassisId = CreateChassis( worldId, position, rotation );
	b3VehicleDef copy = *def;
	copy.chassisId = chassisId;
	b3VehicleId vehicleId = b3CreateVehicle( worldId, &copy );
	ENSURE( b3Vehicle_IsValid( vehicleId ) );
	b3Vehicle_SetInput( vehicleId, input );

	for ( int i = 0; i < 300; ++i )
	{
		Step( worldId, 1 );
		if ( VehicleIsFinite( vehicleId ) == false )
		{
			printf( "    non-finite at step %d\n", i );
			b3DestroyWorld( worldId );
			return 1;
		}
	}

	b3DestroyWorld( worldId );
	return 0;
}

static int RobustnessTest( void )
{
	b3VehicleInput full = { .forward = 1.0f, .steer = 1.0f, .handbrake = 1.0f };
	b3VehicleDef def = b3DefaultVehicleDef();

	// Airborne and upside down
	ENSURE( RunNanScenario( &def, (b3Pos){ 0.0f, 20.0f, 0.0f }, b3Quat_identity, full ) == 0 );
	b3Quat flipped = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, B3_PI );
	ENSURE( RunNanScenario( &def, (b3Pos){ 0.0f, 1.0f, 0.0f }, flipped, full ) == 0 );

	// Zero parameters
	b3VehicleDef zero = b3DefaultVehicleDef();
	zero.chassis.mass = 0.0f;
	zero.engine.maxTorque = 0.0f;
	zero.engine.inertia = 0.0f;
	zero.transmission.clutchStrength = 0.0f;
	zero.transmission.gearCount = 0;
	zero.transmission.finalDrive = 0.0f;
	zero.steering.speedReference = 0.0f;
	zero.drivetrain.differentialCount = 0;
	for ( int i = 0; i < zero.wheelCount; ++i )
	{
		b3VehicleWheelDef* w = zero.wheels + i;
		w->radius = 0.0f;
		w->width = 0.0f;
		w->inertia = 0.0f;
		w->suspension.frequency = 0.0f;
		w->suspension.compressionDamping = 0.0f;
		w->suspension.reboundDamping = 0.0f;
		w->suspension.minLength = 0.0f;
		w->suspension.maxLength = 0.0f;
		w->tire.longitudinalPeakSlip = 0.0f;
		w->tire.lateralPeakAngle = 0.0f;
		w->tire.longitudinalPeakFriction = 0.0f;
		w->tire.lateralPeakFriction = 0.0f;
	}
	ENSURE( RunNanScenario( &zero, (b3Pos){ 0.0f, 1.0f, 0.0f }, b3Quat_identity, full ) == 0 );

	// Extreme parameters
	b3VehicleDef extreme = b3DefaultVehicleDef();
	extreme.chassis.mass = 1.0e5f;
	extreme.engine.maxTorque = 1.0e6f;
	extreme.transmission.clutchStrength = 1.0e6f;
	extreme.assists.downforce = 1.0e3f;
	for ( int i = 0; i < extreme.wheelCount; ++i )
	{
		b3VehicleWheelDef* w = extreme.wheels + i;
		w->suspension.frequency = 50.0f;
		w->suspension.compressionDamping = 10.0f;
		w->maxBrakeTorque = 1.0e7f;
		w->inertia = 1.0e-3f;
		w->tire.lateralPeakFriction = 20.0f;
		w->tire.longitudinalPeakFriction = 20.0f;
	}
	ENSURE( RunNanScenario( &extreme, (b3Pos){ 0.0f, 1.0f, 0.0f }, b3Quat_identity, full ) == 0 );

	// Garbage values are sanitized
	b3VehicleDef garbage = b3DefaultVehicleDef();
	garbage.wheels[0].radius = NAN;
	garbage.wheels[1].suspension.frequency = INFINITY;
	garbage.engine.maxRpm = -5.0f;
	garbage.chassis.up = b3Vec3_zero;
	ENSURE( RunNanScenario( &garbage, (b3Pos){ 0.0f, 1.0f, 0.0f }, b3Quat_identity, full ) == 0 );

	return 0;
}

static uint32_t HashBytes( uint32_t hash, const void* data, int size )
{
	const uint8_t* bytes = data;
	for ( int i = 0; i < size; ++i )
	{
		hash = ( hash ^ bytes[i] ) * 16777619u;
	}
	return hash;
}

static uint32_t DeterminismRun( int workerCount )
{
	b3WorldId worldId = CreateVehicleWorld( workerCount );
	CreateFlatGround( worldId );

	// Loose boxes to drive through
	for ( int i = 0; i < 20; ++i )
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = (b3Pos){ -3.0f + 0.3f * ( i % 5 ), 0.25f + 0.5f * ( i / 5 ), 25.0f };
		b3BodyId boxId = b3CreateBody( worldId, &bodyDef );
		b3BoxHull box = b3MakeCubeHull( 0.25f );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3CreateHullShape( boxId, &shapeDef, &box.base );
	}

	b3VehicleId vehicles[3];
	for ( int i = 0; i < 3; ++i )
	{
		vehicles[i] = CreateCar( worldId, (b3Pos){ -4.0f + 4.0f * i, 0.6f, 0.0f }, NULL );
	}

	uint32_t hash = 2166136261u;
	for ( int i = 0; i < 600; ++i )
	{
		for ( int j = 0; j < 3; ++j )
		{
			float t = i * VEHICLE_DT;
			b3VehicleInput input = { .forward = 1.0f, .steer = 0.6f * sinf( 0.7f * t + j ), .handbrake = ( i / 90 ) % 4 == 3 };
			b3Vehicle_SetInput( vehicles[j], input );
		}
		Step( worldId, 1 );
	}

	for ( int j = 0; j < 3; ++j )
	{
		b3WorldTransform xf = b3Body_GetTransform( b3Vehicle_GetChassisBody( vehicles[j] ) );
		hash = HashBytes( hash, &xf, sizeof( xf ) );
		b3VehicleState state = b3Vehicle_GetState( vehicles[j] );
		hash = HashBytes( hash, &state.engineRpm, sizeof( float ) );
	}

	b3DestroyWorld( worldId );
	return hash;
}

static int DeterminismTest( void )
{
	uint32_t reference = DeterminismRun( 1 );
	for ( int workerCount = 2; workerCount <= 5; ++workerCount )
	{
		uint32_t hash = DeterminismRun( workerCount );
		printf( "    workers=%d hash=0x%08X\n", workerCount, hash );
		ENSURE( hash == reference );
	}
	return 0;
}

static int DestroyTest( void )
{
	b3WorldId worldId = CreateVehicleWorld( 1 );
	CreateFlatGround( worldId );
	b3VehicleId a = CreateCar( worldId, (b3Pos){ 0.0f, 0.6f, 0.0f }, NULL );
	b3VehicleId b = CreateCar( worldId, (b3Pos){ 5.0f, 0.6f, 0.0f }, NULL );
	Step( worldId, 10 );
	ENSURE( b3World_GetVehicleCount( worldId ) == 2 );

	b3BodyId chassisA = b3Vehicle_GetChassisBody( a );
	b3DestroyBody( chassisA );
	ENSURE( b3Vehicle_IsValid( a ) == false );
	ENSURE( b3Vehicle_IsValid( b ) );
	ENSURE( b3World_GetVehicleCount( worldId ) == 1 );
	Step( worldId, 10 );

	b3BodyId chassisB = b3Vehicle_GetChassisBody( b );
	ENSURE( b3Body_GetJointCount( chassisB ) == 4 );
	b3DestroyVehicle( b );
	ENSURE( b3Vehicle_IsValid( b ) == false );
	ENSURE( b3Body_GetJointCount( chassisB ) == 0 );
	ENSURE( b3World_GetVehicleCount( worldId ) == 0 );
	Step( worldId, 10 );

	// Slots are reused with a new generation
	b3VehicleId c = CreateCar( worldId, (b3Pos){ 0.0f, 0.6f, 5.0f }, NULL );
	ENSURE( b3Vehicle_IsValid( c ) );
	ENSURE( b3Vehicle_IsValid( a ) == false );
	Step( worldId, 10 );

	// Destroying the world with live vehicles must not leak
	b3DestroyWorld( worldId );
	return 0;
}

static float RideHeightAt( float dt, int substeps )
{
	b3WorldId worldId = CreateVehicleWorld( 1 );
	CreateFlatGround( worldId );
	b3VehicleId vehicleId = CreateCar( worldId, (b3Pos){ 0.0f, 0.8f, 0.0f }, NULL );
	b3BodyId chassisId = b3Vehicle_GetChassisBody( vehicleId );
	int steps = (int)( 4.0f / dt );
	for ( int i = 0; i < steps; ++i )
	{
		b3World_Step( worldId, dt, substeps );
	}

	// Drive a little to check the drivetrain too
	b3Vehicle_SetInput( vehicleId, (b3VehicleInput){ .forward = 1.0f, .steer = 0.3f } );
	for ( int i = 0; i < steps; ++i )
	{
		b3World_Step( worldId, dt, substeps );
	}
	// Brake, then hold with the handbrake near a stop, since the brake selects reverse once stopped
	for ( int i = 0; i < 2 * steps; ++i )
	{
		bool fast = b3Vehicle_GetState( vehicleId ).speed > 2.0f;
		b3Vehicle_SetInput( vehicleId, (b3VehicleInput){ .brake = fast ? 1.0f : 0.0f, .handbrake = fast ? 0.0f : 1.0f } );
		b3World_Step( worldId, dt, substeps );
	}

	float height = (float)b3Body_GetPosition( chassisId ).y;
	float speed = b3Length( b3Body_GetLinearVelocity( chassisId ) );
	bool finite = VehicleIsFinite( vehicleId );
	b3DestroyWorld( worldId );
	if ( finite == false || speed > 0.05f )
	{
		return -1.0f;
	}
	return height;
}

static int TimeStepTest( void )
{
	float h30 = RideHeightAt( 1.0f / 30.0f, 4 );
	float h60 = RideHeightAt( 1.0f / 60.0f, 4 );
	float h120 = RideHeightAt( 1.0f / 120.0f, 4 );
	printf( "    ride height at 30 Hz %.3f, 60 Hz %.3f, 120 Hz %.3f\n", h30, h60, h120 );
	ENSURE( h30 > 0.0f && h60 > 0.0f && h120 > 0.0f );
	ENSURE( fabsf( h30 - h60 ) < 0.02f );
	ENSURE( fabsf( h120 - h60 ) < 0.02f );
	return 0;
}

static int LiveSettingsTest( void )
{
	b3WorldId worldId = CreateVehicleWorld( 1 );
	CreateFlatGround( worldId );
	b3VehicleId vehicleId = CreateCar( worldId, (b3Pos){ 0.0f, 0.6f, 0.0f }, NULL );
	b3BodyId chassisId = b3Vehicle_GetChassisBody( vehicleId );
	Step( worldId, 180 );
	float before = (float)b3Body_GetPosition( chassisId ).y;

	// Preload raises the car
	for ( int i = 0; i < 4; ++i )
	{
		b3VehicleWheelDef wheel = b3Vehicle_GetWheel( vehicleId, i );
		wheel.suspension.preload = 0.05f;
		b3Vehicle_SetWheel( vehicleId, i, wheel );
	}
	Step( worldId, 180 );
	float after = (float)b3Body_GetPosition( chassisId ).y;
	printf( "    preload height %.3f -> %.3f\n", before, after );
	ENSURE( after > before + 0.03f );

	// Mass change updates the sprung masses
	b3VehicleChassis chassis = b3Vehicle_GetChassis( vehicleId );
	chassis.mass = 2000.0f;
	b3Vehicle_SetChassis( vehicleId, chassis );
	Step( worldId, 1 );
	ENSURE_SMALL( b3Body_GetMass( chassisId ) - 2000.0f, 0.5f );
	float total = 0.0f;
	for ( int i = 0; i < 4; ++i )
	{
		total += b3Vehicle_GetSprungMass( vehicleId, i );
	}
	ENSURE_SMALL( total - 2000.0f, 1.0f );

	// Manual gearbox follows b3Vehicle_SetGear
	b3VehicleTransmission t = b3Vehicle_GetTransmission( vehicleId );
	t.mode = b3_vehicleTransmissionManual;
	b3Vehicle_SetTransmission( vehicleId, t );
	b3Vehicle_SetGear( vehicleId, 2 );
	b3Vehicle_SetInput( vehicleId, (b3VehicleInput){ .forward = 1.0f } );
	Step( worldId, 600 );
	ENSURE( b3Vehicle_GetState( vehicleId ).gear == 2 );

	b3Vehicle_SetGear( vehicleId, -1 );
	b3Vehicle_SetInput( vehicleId, (b3VehicleInput){ .brake = 1.0f } );
	Step( worldId, 300 );
	b3Vehicle_SetInput( vehicleId, (b3VehicleInput){ .forward = 1.0f } );
	Step( worldId, 120 );
	ENSURE( b3Vehicle_GetState( vehicleId ).speed < -1.0f );

	// Reset puts the car back upright and still
	b3Vehicle_Reset( vehicleId, (b3Pos){ 0.0f, 0.6f, 0.0f }, b3Quat_identity );
	b3Vehicle_SetInput( vehicleId, (b3VehicleInput){ 0 } );
	ENSURE( b3Length( b3Body_GetLinearVelocity( chassisId ) ) == 0.0f );
	Step( worldId, 120 );
	ENSURE( VehicleIsFinite( vehicleId ) );

	b3DestroyWorld( worldId );
	return 0;
}

static int SleepTest( void )
{
	b3WorldId worldId = CreateVehicleWorld( 1 );
	CreateFlatGround( worldId );
	b3VehicleId vehicleId = CreateCar( worldId, (b3Pos){ 0.0f, 0.6f, 0.0f }, NULL );
	b3BodyId chassisId = b3Vehicle_GetChassisBody( vehicleId );
	Step( worldId, 300 );
	ENSURE( b3Body_IsAwake( chassisId ) == false );

	b3Vehicle_SetInput( vehicleId, (b3VehicleInput){ .forward = 1.0f } );
	ENSURE( b3Body_IsAwake( chassisId ) );
	Step( worldId, 60 );
	ENSURE( b3Vehicle_GetState( vehicleId ).speed > 1.0f );

	// Brake, then hold with the handbrake near a stop, since the brake selects reverse once stopped
	for ( int i = 0; i < 120; ++i )
	{
		bool fast = b3Vehicle_GetState( vehicleId ).speed > 2.0f;
		b3Vehicle_SetInput( vehicleId, (b3VehicleInput){ .brake = fast ? 1.0f : 0.0f, .handbrake = fast ? 0.0f : 1.0f } );
		Step( worldId, 1 );
	}
	b3Vehicle_SetInput( vehicleId, (b3VehicleInput){ 0 } );
	Step( worldId, 300 );
	ENSURE( b3Body_IsAwake( chassisId ) == false );

	b3DestroyWorld( worldId );
	return 0;
}

// Reads the bulk wheel data and checks every wheel against the per wheel getters. Returns 0 when
// every check passes, and the number of wheels with each flag set through the counts.
static int CheckWheelData( b3VehicleId vehicleId, b3VehicleWheelData* data, int* contactCount, int* stickyCount )
{
	int count = b3Vehicle_GetWheelData( vehicleId, data, B3_MAX_VEHICLE_WHEELS );
	ENSURE( count == b3Vehicle_GetWheelCount( vehicleId ) );

	*contactCount = 0;
	*stickyCount = 0;
	for ( int i = 0; i < count; ++i )
	{
		b3VehicleWheelState ws = b3Vehicle_GetWheelState( vehicleId, i );
		b3WorldTransform xf = b3Vehicle_GetWheelTransform( vehicleId, i );
		const b3VehicleWheelData* d = data + i;

		ENSURE_SMALL( d->position.x - (float)xf.p.x, 1e-4f );
		ENSURE_SMALL( d->position.y - (float)xf.p.y, 1e-4f );
		ENSURE_SMALL( d->position.z - (float)xf.p.z, 1e-4f );
		ENSURE( d->rotation.v.x == xf.q.v.x && d->rotation.v.y == xf.q.v.y && d->rotation.v.z == xf.q.v.z &&
				d->rotation.s == xf.q.s );
		ENSURE( ( ( d->flags & b3_vehicleWheelContact ) != 0 ) == ws.hasContact );
		ENSURE( ( ( d->flags & b3_vehicleWheelSticky ) != 0 ) == ws.isSticky );
		ENSURE( ( d->flags & ~( b3_vehicleWheelContact | b3_vehicleWheelSticky ) ) == 0 );
		ENSURE( d->contactPoint.x == (float)ws.contactPoint.x && d->contactPoint.y == (float)ws.contactPoint.y &&
				d->contactPoint.z == (float)ws.contactPoint.z );
		ENSURE( d->contactNormal.x == ws.contactNormal.x && d->contactNormal.y == ws.contactNormal.y &&
				d->contactNormal.z == ws.contactNormal.z );
		ENSURE( d->suspensionLength == ws.suspensionLength && d->compression == ws.compression );
		ENSURE( d->load == ws.load && d->slipRatio == ws.slipRatio && d->slipAngle == ws.slipAngle );
		ENSURE( d->angularVelocity == ws.angularVelocity && d->rotationAngle == ws.rotationAngle );
		ENSURE( d->steerAngle == ws.steerAngle );
		ENSURE( d->longitudinalForce == ws.longitudinalForce && d->lateralForce == ws.lateralForce );
		ENSURE( d->driveTorque == ws.driveTorque && d->brakeTorque == ws.brakeTorque );
		ENSURE( B3_ID_EQUALS( d->groundShapeId, ws.groundShapeId ) );
		ENSURE( d->userMaterialId == ws.userMaterialId );

		*contactCount += ws.hasContact ? 1 : 0;
		*stickyCount += ws.isSticky ? 1 : 0;
	}

	return 0;
}

// The bulk wheel readout matches the per wheel getters parked, driving and airborne
static int WheelDataTest( void )
{
	b3WorldId worldId = CreateVehicleWorld( 1 );
	CreateFlatGround( worldId );
	b3VehicleId vehicleId = CreateCar( worldId, (b3Pos){ 0.0f, 1.0f, 0.0f }, NULL );
	b3BodyId chassisId = b3Vehicle_GetChassisBody( vehicleId );

	b3VehicleWheelData data[B3_MAX_VEHICLE_WHEELS];
	int contactCount, stickyCount;

	// Parked: the sticky tires hold the car
	Step( worldId, 180 );
	ENSURE( CheckWheelData( vehicleId, data, &contactCount, &stickyCount ) == 0 );
	ENSURE( contactCount == 4 && stickyCount == 4 );

	// Driving and turning: the front wheels are steered
	b3Vehicle_SetInput( vehicleId, (b3VehicleInput){ .forward = 1.0f, .steer = 0.5f } );
	Step( worldId, 60 );
	ENSURE( CheckWheelData( vehicleId, data, &contactCount, &stickyCount ) == 0 );
	ENSURE( contactCount == 4 && stickyCount == 0 );
	ENSURE( data[0].steerAngle != 0.0f && data[0].load > 0.0f );
	ENSURE( B3_IS_NON_NULL( data[0].groundShapeId ) );

	// Lifted off the ground: no stale contact is reported. Loads and forces are read back from the
	// previous step's solve, so they clear one step after the contact does.
	b3Body_SetTransform( chassisId, (b3Pos){ 0.0f, 50.0f, 0.0f }, b3Quat_identity );
	Step( worldId, 2 );
	ENSURE( CheckWheelData( vehicleId, data, &contactCount, &stickyCount ) == 0 );
	ENSURE( contactCount == 0 && stickyCount == 0 );
	for ( int i = 0; i < 4; ++i )
	{
		ENSURE( data[i].flags == 0 );
		ENSURE( data[i].contactPoint.x == 0.0f && data[i].contactPoint.y == 0.0f && data[i].contactPoint.z == 0.0f );
		ENSURE( data[i].contactNormal.x == 0.0f && data[i].contactNormal.y == 0.0f && data[i].contactNormal.z == 0.0f );
		ENSURE( B3_IS_NULL( data[i].groundShapeId ) && data[i].userMaterialId == 0 );
		ENSURE( data[i].load == 0.0f );
	}

	// Capacity limits the write, and a negative capacity writes nothing
	b3VehicleWheelData partial[3] = { 0 };
	partial[2].load = -1.0f;
	ENSURE( b3Vehicle_GetWheelData( vehicleId, partial, 2 ) == 2 );
	ENSURE( partial[2].load == -1.0f );
	ENSURE( b3Vehicle_GetWheelData( vehicleId, partial, -1 ) == 0 );

	b3DestroyWorld( worldId );
	return 0;
}

// Speed after one second of full throttle on a flat mesh whose right half (x > 0), where the car
// sits, uses a second material with the given friction. The left half keeps friction 0.6.
static float MeshMaterialLaunchSpeed( float rightFriction )
{
	b3WorldId worldId = CreateVehicleWorld( 1 );

	b3Vec3 vertices[6] = {
		{ -200.0f, 0.0f, -200.0f }, { 0.0f, 0.0f, -200.0f }, { 200.0f, 0.0f, -200.0f },
		{ -200.0f, 0.0f, 200.0f },	{ 0.0f, 0.0f, 200.0f },	 { 200.0f, 0.0f, 200.0f },
	};
	int32_t indices[12] = { 0, 3, 4, 0, 4, 1, 1, 4, 5, 1, 5, 2 };
	uint8_t materialIndices[4] = { 0, 0, 1, 1 };

	b3MeshDef meshDef = { 0 };
	meshDef.vertices = vertices;
	meshDef.indices = indices;
	meshDef.materialIndices = materialIndices;
	meshDef.vertexCount = 6;
	meshDef.triangleCount = 4;
	b3MeshData* mesh = b3CreateMesh( &meshDef, NULL, 0 );

	b3SurfaceMaterial materials[2] = { b3DefaultSurfaceMaterial(), b3DefaultSurfaceMaterial() };
	materials[0].friction = 0.6f;
	materials[1].friction = rightFriction;

	b3BodyDef bodyDef = b3DefaultBodyDef();
	b3BodyId groundId = b3CreateBody( worldId, &bodyDef );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.materials = materials;
	shapeDef.materialCount = 2;
	b3CreateMeshShape( groundId, &shapeDef, mesh, (b3Vec3){ 1.0f, 1.0f, 1.0f } );

	b3VehicleId vehicleId = CreateCar( worldId, (b3Pos){ 20.0f, 0.6f, 0.0f }, NULL );
	Step( worldId, 120 );
	b3Vehicle_SetInput( vehicleId, (b3VehicleInput){ .forward = 1.0f } );
	Step( worldId, 60 );
	float speed = b3Vehicle_GetState( vehicleId ).speed;

	b3DestroyWorld( worldId );
	b3DestroyMesh( mesh );
	return speed;
}

// Wheels use the friction of the triangle they touch, not the shape's first material
static int MeshMaterialTest( void )
{
	float grip = MeshMaterialLaunchSpeed( 0.6f );
	float ice = MeshMaterialLaunchSpeed( 0.1f );
	printf( "    speed after 1 s: %.2f m/s on friction 0.6, %.2f m/s on friction 0.1\n", grip, ice );
	ENSURE( grip > 3.0f );
	ENSURE( ice < 0.5f * grip );
	return 0;
}

// With ABS on, the service brake holds on the first step a wheel touches down, before the solver
// has reported any load for it
static int TouchdownBrakeTest( void )
{
	b3WorldId worldId = CreateVehicleWorld( 1 );
	CreateFlatGround( worldId );
	b3VehicleDef def = b3DefaultVehicleDef();
	def.assists.abs = true;
	b3VehicleId vehicleId = CreateCar( worldId, (b3Pos){ 0.0f, 2.0f, 0.0f }, &def );

	// Moving, so the sticky tire does not engage and hold the wheels instead of ABS
	b3Body_SetLinearVelocity( b3Vehicle_GetChassisBody( vehicleId ), (b3Vec3){ 0.0f, 0.0f, 15.0f } );
	b3Vehicle_SetInput( vehicleId, (b3VehicleInput){ .brake = 1.0f } );

	bool touched = false;
	for ( int i = 0; i < 120 && touched == false; ++i )
	{
		Step( worldId, 1 );
		for ( int j = 0; j < 4; ++j )
		{
			b3VehicleWheelState ws = b3Vehicle_GetWheelState( vehicleId, j );
			if ( ws.hasContact )
			{
				printf( "    wheel %d first contact: load %.0f N, brake %.0f N*m, sticky %d\n", j, ws.load, ws.brakeTorque,
						ws.isSticky );
				ENSURE( ws.isSticky == false );
				ENSURE( ws.brakeTorque > 500.0f );
				touched = true;
			}
		}
	}

	ENSURE( touched );
	b3DestroyWorld( worldId );
	return 0;
}

// With the automatic gearbox and the brake reverse assist, the brake slows the car in a forward
// gear, then selects reverse and drives backward once stopped
static int BrakeReverseTest( void )
{
	b3WorldId worldId = CreateVehicleWorld( 1 );
	CreateFlatGround( worldId );
	b3VehicleId vehicleId = CreateCar( worldId, (b3Pos){ 0.0f, 0.6f, 0.0f }, NULL );
	Step( worldId, 60 );

	b3Vehicle_SetInput( vehicleId, (b3VehicleInput){ .forward = 1.0f } );
	Step( worldId, 180 );
	ENSURE( b3Vehicle_GetState( vehicleId ).speed > 5.0f );

	b3Vehicle_SetInput( vehicleId, (b3VehicleInput){ .brake = 1.0f } );
	bool reversed = false;
	float minSpeed = FLT_MAX;
	for ( int i = 0; i < 600; ++i )
	{
		Step( worldId, 1 );
		b3VehicleState state = b3Vehicle_GetState( vehicleId );
		minSpeed = b3MinFloat( minSpeed, state.speed );
		if ( state.speed > 2.0f )
		{
			// Still braking, not driving
			ENSURE( state.gear > 0 );
			ENSURE( state.brake == 1.0f && state.throttle == 0.0f );
		}
		else if ( state.gear < 0 )
		{
			reversed = true;
		}
	}

	b3VehicleState state = b3Vehicle_GetState( vehicleId );
	printf( "    brake reverse: gear %d, speed %.2f m/s, throttle %.2f\n", state.gear, minSpeed, state.throttle );
	ENSURE( reversed );
	ENSURE( state.gear == -1 );
	ENSURE( state.brake == 0.0f && state.throttle == 1.0f );
	ENSURE( minSpeed < -2.0f );

	// With forward input the brake stays a brake
	b3Vehicle_SetInput( vehicleId, (b3VehicleInput){ .forward = 1.0f, .brake = 1.0f } );
	Step( worldId, 1 );
	ENSURE( b3Vehicle_GetState( vehicleId ).brake == 1.0f );

	// With the assist off the brake holds the car still
	b3VehicleAssists assists = b3Vehicle_GetAssists( vehicleId );
	ENSURE( assists.brakeReverse );
	assists.brakeReverse = false;
	b3Vehicle_SetAssists( vehicleId, assists );
	b3Vehicle_Reset( vehicleId, (b3Pos){ 0.0f, 0.6f, 0.0f }, b3Quat_identity );
	b3Vehicle_SetInput( vehicleId, (b3VehicleInput){ .brake = 1.0f } );
	Step( worldId, 180 );
	state = b3Vehicle_GetState( vehicleId );
	printf( "    brake reverse off: gear %d, speed %.3f m/s\n", state.gear, state.speed );
	ENSURE( state.gear > 0 );
	ENSURE( state.brake == 1.0f && state.throttle == 0.0f );
	ENSURE( fabsf( state.speed ) < 0.05f );

	ENSURE( VehicleIsFinite( vehicleId ) );
	b3DestroyWorld( worldId );
	return 0;
}

static b3VehicleTurbo SingleTurbo( void )
{
	b3VehicleTurbo turbo = { 0 };
	turbo.stages[0] = (b3VehicleTurboStage){
		.gain = 0.5f,
		.spoolRpm = 2000.0f,
		.fullBoostRpm = 3500.0f,
		.spoolTime = 0.8f,
		.bleedTime = 0.15f,
	};
	return turbo;
}

// Full throttle to 100 km/h. Returns the time, and the lowest boost seen after the boost first
// came within 90% of the full gain, which is where the shifts cut the throttle.
static float TurboLaunch( const b3VehicleTurbo* turbo, float* peakBoost, float* minBoostAfterPeak )
{
	b3WorldId worldId = CreateVehicleWorld( 1 );
	CreateFlatGround( worldId );
	b3VehicleDef def = b3DefaultVehicleDef();
	if ( turbo != NULL )
	{
		def.turbo = *turbo;
	}
	b3VehicleId vehicleId = CreateCar( worldId, (b3Pos){ 0.0f, 0.6f, 0.0f }, &def );
	Step( worldId, 120 );

	float gain = turbo != NULL ? turbo->stages[0].gain + turbo->stages[1].gain : 0.0f;
	*peakBoost = 0.0f;
	*minBoostAfterPeak = FLT_MAX;
	b3Vehicle_SetInput( vehicleId, (b3VehicleInput){ .forward = 1.0f } );
	float time = -1.0f;
	for ( int i = 0; i < 30 * 60; ++i )
	{
		Step( worldId, 1 );
		b3VehicleState state = b3Vehicle_GetState( vehicleId );
		*peakBoost = b3MaxFloat( *peakBoost, state.boost );
		if ( gain > 0.0f && *peakBoost > 0.9f * gain )
		{
			*minBoostAfterPeak = b3MinFloat( *minBoostAfterPeak, state.boost );
		}
		if ( state.speed >= 100.0f * KMH )
		{
			time = ( i + 1 ) * VEHICLE_DT;
			break;
		}
	}

	if ( VehicleIsFinite( vehicleId ) == false )
	{
		time = -1.0f;
	}

	b3DestroyWorld( worldId );
	return time;
}

// Revs the engine in neutral to the full boost rpm, holds full throttle for the given time, then
// lifts for the given time and returns the boost
static float RevBoost( const b3VehicleTurbo* turbo, float throttleTime, float liftTime )
{
	b3WorldId worldId = CreateVehicleWorld( 1 );
	CreateFlatGround( worldId );
	b3VehicleDef def = b3DefaultVehicleDef();
	def.turbo = *turbo;
	def.transmission.mode = b3_vehicleTransmissionManual;
	b3VehicleId vehicleId = CreateCar( worldId, (b3Pos){ 0.0f, 0.6f, 0.0f }, &def );
	Step( worldId, 60 );
	b3Vehicle_SetGear( vehicleId, 0 );
	Step( worldId, 30 );

	b3Vehicle_SetInput( vehicleId, (b3VehicleInput){ .forward = 1.0f } );
	for ( int i = 0; i < 120 && b3Vehicle_GetState( vehicleId ).engineRpm < turbo->stages[0].fullBoostRpm; ++i )
	{
		Step( worldId, 1 );
	}
	Step( worldId, (int)( throttleTime / VEHICLE_DT + 0.5f ) );
	b3Vehicle_SetInput( vehicleId, (b3VehicleInput){ .handbrake = 1.0f } );
	Step( worldId, (int)( liftTime / VEHICLE_DT + 0.5f ) );
	float boost = b3Vehicle_GetState( vehicleId ).boost;

	b3DestroyWorld( worldId );
	return boost;
}

// Revs the engine in neutral with the pedal held at the given position and returns the boost
// after the given time
static float NeutralBoost( const b3VehicleTurbo* turbo, float pedal, float time )
{
	b3WorldId worldId = CreateVehicleWorld( 1 );
	CreateFlatGround( worldId );
	b3VehicleDef def = b3DefaultVehicleDef();
	def.turbo = *turbo;
	def.transmission.mode = b3_vehicleTransmissionManual;
	b3VehicleId vehicleId = CreateCar( worldId, (b3Pos){ 0.0f, 0.6f, 0.0f }, &def );
	Step( worldId, 60 );
	b3Vehicle_SetGear( vehicleId, 0 );
	Step( worldId, 30 );
	b3Vehicle_SetInput( vehicleId, (b3VehicleInput){ .forward = pedal } );
	Step( worldId, (int)( time / VEHICLE_DT + 0.5f ) );
	float boost = b3Vehicle_GetState( vehicleId ).boost;
	b3DestroyWorld( worldId );
	return boost;
}

// Spools the turbo fully in neutral, eases the pedal below the boost gate for the given time so
// the engine keeps revving with no boost asked for, then opens it again for one step. Returns
// the boost while eased and after reopening.
static void LiftAndReopen( const b3VehicleTurbo* turbo, float liftTime, float* eased, float* reopened )
{
	b3WorldId worldId = CreateVehicleWorld( 1 );
	CreateFlatGround( worldId );
	b3VehicleDef def = b3DefaultVehicleDef();
	def.turbo = *turbo;
	def.transmission.mode = b3_vehicleTransmissionManual;
	b3VehicleId vehicleId = CreateCar( worldId, (b3Pos){ 0.0f, 0.6f, 0.0f }, &def );
	Step( worldId, 60 );
	b3Vehicle_SetGear( vehicleId, 0 );
	Step( worldId, 30 );
	b3Vehicle_SetInput( vehicleId, (b3VehicleInput){ .forward = 1.0f } );
	Step( worldId, 180 );
	b3Vehicle_SetInput( vehicleId, (b3VehicleInput){ .forward = 0.5f } );
	Step( worldId, (int)( liftTime / VEHICLE_DT + 0.5f ) );
	*eased = b3Vehicle_GetState( vehicleId ).boost;
	b3Vehicle_SetInput( vehicleId, (b3VehicleInput){ .forward = 1.0f } );
	Step( worldId, 1 );
	*reopened = b3Vehicle_GetState( vehicleId ).boost;
	b3DestroyWorld( worldId );
}

// Time from full throttle in neutral until the boost reaches 80% of the gain, for an engine with
// the given rev limit
static float SpoolTimeAt( const b3VehicleTurbo* turbo, float maxRpm )
{
	b3WorldId worldId = CreateVehicleWorld( 1 );
	CreateFlatGround( worldId );
	b3VehicleDef def = b3DefaultVehicleDef();
	def.turbo = *turbo;
	def.engine.maxRpm = maxRpm;
	def.transmission.mode = b3_vehicleTransmissionManual;
	b3VehicleId vehicleId = CreateCar( worldId, (b3Pos){ 0.0f, 0.6f, 0.0f }, &def );
	Step( worldId, 60 );
	b3Vehicle_SetGear( vehicleId, 0 );
	Step( worldId, 30 );
	b3Vehicle_SetInput( vehicleId, (b3VehicleInput){ .forward = 1.0f } );
	float time = -1.0f;
	for ( int i = 0; i < 600; ++i )
	{
		Step( worldId, 1 );
		if ( b3Vehicle_GetState( vehicleId ).boost >= 0.8f * turbo->stages[0].gain )
		{
			time = ( i + 1 ) * VEHICLE_DT;
			break;
		}
	}
	b3DestroyWorld( worldId );
	return time;
}

static int TurboTest( void )
{
	// Naturally aspirated by default
	b3VehicleDef def = b3DefaultVehicleDef();
	ENSURE( def.turbo.stages[0].gain == 0.0f && def.turbo.stages[1].gain == 0.0f );
	float naPeak, naMin;
	float naTime = TurboLaunch( NULL, &naPeak, &naMin );
	ENSURE( naTime > 0.0f );
	ENSURE( naPeak == 0.0f );

	// The same engine with a turbo is quicker, and its boost reaches the full gain. Traction control
	// holds both cars to the tire grip in first gear, so the gain shows from second gear on.
	b3VehicleTurbo turbo = SingleTurbo();
	float turboPeak, turboMin;
	float turboTime = TurboLaunch( &turbo, &turboPeak, &turboMin );
	printf( "    0-100 km/h: na %.2f s, turbo %.2f s, peak boost %.2f\n", naTime, turboTime, turboPeak );
	ENSURE( turboTime > 0.0f && turboTime < 0.98f * naTime );
	ENSURE( turboPeak > 0.45f && turboPeak <= 0.5f + 1e-4f );

	// Shifts cut the throttle and the boost bleeds off. Anti-lag holds it through the shifts.
	b3VehicleTurbo antiLag = turbo;
	antiLag.antiLag = true;
	float antiLagPeak, antiLagMin;
	TurboLaunch( &antiLag, &antiLagPeak, &antiLagMin );
	printf( "    boost through shifts: %.2f, with anti-lag %.2f\n", turboMin, antiLagMin );
	ENSURE( turboMin < 0.8f * 0.5f );
	ENSURE( antiLagMin > 0.9f * 0.5f );

	// Lag: the turbo is still spooling a quarter second after the engine reached full boost rpm,
	// while a supercharger is there at once
	float lagged = RevBoost( &turbo, 0.25f, 0.0f );
	float spooled = RevBoost( &turbo, 2.0f, 0.0f );
	b3VehicleTurbo supercharger = turbo;
	supercharger.instantFraction = 1.0f;
	float instant = RevBoost( &supercharger, 0.25f, 0.0f );
	printf( "    boost after 0.25 s: turbo %.2f, supercharger %.2f, after 2 s turbo %.2f\n", lagged, instant, spooled );
	ENSURE( lagged < 0.5f * 0.5f );
	ENSURE( spooled > 0.9f * 0.5f );
	ENSURE( instant > 0.9f * 0.5f );

	// Lifting dumps the boost through the blow-off valve
	float bled = RevBoost( &turbo, 2.0f, 0.5f );
	ENSURE( bled < 0.05f );

	// but the shaft keeps spinning: with a long bleed time the boost is back the moment the pedal
	// opens again after a short lift, with a short one the turbo has to spool up again
	b3VehicleTurbo slowBleed = turbo;
	slowBleed.stages[0].bleedTime = 1.5f;
	float easedSlow, reopenedSlow, easedFast, reopenedFast;
	LiftAndReopen( &slowBleed, 0.3f, &easedSlow, &reopenedSlow );
	LiftAndReopen( &turbo, 0.3f, &easedFast, &reopenedFast );
	printf( "    after a 0.3 s lift: boost %.2f, reopened %.2f with a 1.5 s bleed, %.2f with 0.15 s\n", easedSlow,
			reopenedSlow, reopenedFast );
	ENSURE( easedSlow < 0.05f && easedFast < 0.05f );
	ENSURE( reopenedSlow > 0.7f * 0.5f );
	ENSURE( reopenedFast < 0.3f * 0.5f );

	// Part pedal: the engine makes the first 1 / 1.5 of the pedal without boost. At 80% the pedal
	// asks for 0.4 of the full boost, and spools toward it slower than at full load.
	float partBoost = NeutralBoost( &turbo, 0.6f, 2.0f );
	float moreBoost = NeutralBoost( &turbo, 0.8f, 2.0f );
	printf( "    boost at 60%% pedal %.2f, at 80%% %.2f\n", partBoost, moreBoost );
	ENSURE( partBoost == 0.0f );
	ENSURE( moreBoost > 0.05f && moreBoost <= 0.4f * 0.5f + 1e-4f );

	// Exhaust energy drives the shaft, so the same turbo spools faster on an engine revving higher
	float lowRev = SpoolTimeAt( &turbo, 4500.0f );
	float highRev = SpoolTimeAt( &turbo, 9000.0f );
	printf( "    time to 80%% boost: %.2f s on a 4500 rpm engine, %.2f s on a 9000 rpm one\n", lowRev, highRev );
	ENSURE( lowRev > 0.0f && highRev > 0.0f && highRev < 0.75f * lowRev );

	// A sequential twin sums its stages
	b3VehicleTurbo twin = turbo;
	twin.stages[0].gain = 0.25f;
	twin.stages[1] = (b3VehicleTurboStage){
		.gain = 0.25f, .spoolRpm = 4000.0f, .fullBoostRpm = 5000.0f, .spoolTime = 1.0f, .bleedTime = 0.2f
	};
	float twinPeak, twinMin;
	ENSURE( TurboLaunch( &twin, &twinPeak, &twinMin ) > 0.0f );
	ENSURE( twinPeak > 0.45f && twinPeak <= 0.5f + 1e-4f );

	// Live settings round-trip and are sanitized
	b3WorldId worldId = CreateVehicleWorld( 1 );
	CreateFlatGround( worldId );
	b3VehicleId vehicleId = CreateCar( worldId, (b3Pos){ 0.0f, 0.6f, 0.0f }, NULL );
	b3VehicleTurbo bad = turbo;
	bad.stages[0].fullBoostRpm = 1000.0f;
	bad.stages[1].gain = NAN;
	bad.instantFraction = 2.0f;
	b3Vehicle_SetTurbo( vehicleId, bad );
	b3VehicleTurbo got = b3Vehicle_GetTurbo( vehicleId );
	ENSURE( got.stages[0].gain == 0.5f );
	ENSURE( got.stages[0].fullBoostRpm == got.stages[0].spoolRpm );
	ENSURE( got.stages[1].gain == 0.0f );
	ENSURE( got.instantFraction == 1.0f );
	b3Vehicle_SetInput( vehicleId, (b3VehicleInput){ .forward = 1.0f } );
	Step( worldId, 120 );
	ENSURE( b3Vehicle_GetState( vehicleId ).boost > 0.0f );
	ENSURE( VehicleIsFinite( vehicleId ) );

	// Reset clears the boost
	b3Vehicle_Reset( vehicleId, (b3Pos){ 0.0f, 0.6f, 0.0f }, b3Quat_identity );
	ENSURE( b3Vehicle_GetState( vehicleId ).boost == 0.0f );

	b3DestroyWorld( worldId );
	return 0;
}

int VehicleTest( void )
{
	RUN_SUBTEST( RideHeightTest );
	RUN_SUBTEST( AccelerationTest );
	RUN_SUBTEST( TractionControlTest );
	RUN_SUBTEST( LaunchControlTest );
	RUN_SUBTEST( SlipRelaxationTest );
	RUN_SUBTEST( BrakingTest );
	RUN_SUBTEST( BrakeTurnTest );
	RUN_SUBTEST( BrakeReverseTest );
	RUN_SUBTEST( TurningTest );
	RUN_SUBTEST( AntiRollTest );
	RUN_SUBTEST( SlopeHoldTest );
	RUN_SUBTEST( DynamicGroundTest );
	RUN_SUBTEST( RampJumpTest );
	RUN_SUBTEST( RobustnessTest );
	RUN_SUBTEST( DeterminismTest );
	RUN_SUBTEST( DestroyTest );
	RUN_SUBTEST( TimeStepTest );
	RUN_SUBTEST( LiveSettingsTest );
	RUN_SUBTEST( SleepTest );
	RUN_SUBTEST( WheelDataTest );
	RUN_SUBTEST( MeshMaterialTest );
	RUN_SUBTEST( TouchdownBrakeTest );
	RUN_SUBTEST( TurboTest );
	return 0;
}
